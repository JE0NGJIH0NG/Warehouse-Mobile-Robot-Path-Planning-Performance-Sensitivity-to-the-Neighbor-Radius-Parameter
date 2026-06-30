#include "frrt/frrt.hpp"

#include <random>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <deque>
#include <limits>

#include <chrono>

#include <nav2_costmap_2d/cost_values.hpp>  // Nav2 비용 임계값 상수

namespace frrt
{

void FRRT::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string /*name*/,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  RCLCPP_INFO(logger_, "[F-RRT*] Algorithm is configured");
}

void FRRT::cleanup()    { RCLCPP_INFO(logger_, "[F-RRT*] cleanup."); }
void FRRT::activate()   { RCLCPP_INFO(logger_, "[F-RRT*] activated"); }
void FRRT::deactivate() { RCLCPP_INFO(logger_, "[F-RRT*] deactivated"); }

// -----------------------------------------------------------------------------

bool FRRT::isCollisionFree(double x0, double y0, double x1, double y1)
{
  unsigned int mx, my;
  const double dist  = std::hypot(x1 - x0, y1 - y0);
  const int    steps = std::max(1, static_cast<int>(dist / 0.05)); // 0 나눗셈 방지

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double x = x0 + (x1 - x0) * t;
    const double y = y0 + (y1 - y0) * t;

    if (!costmap_ros_->getCostmap()->worldToMap(x, y, mx, my)) return false;
    const unsigned char c = costmap_ros_->getCostmap()->getCost(mx, my);
    if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false; // Nav2 기준
  }
  return true;
}

std::vector<std::pair<size_t,double>>
FRRT::getNear(KDTree& kdtree, double x, double y, double r,
              size_t exclude_idx, bool sort_by_distance)
{
  std::vector<std::pair<size_t,double>> matches;
  matches.reserve(64);

  nanoflann::RadiusResultSet<double, size_t> rs(r*r, matches);
  rs.init();
  double q[2] = { x, y };

  nanoflann::SearchParams params(10);
  params.sorted = sort_by_distance;
  kdtree.findNeighbors(rs, q, params);   // matches: {index, squared_distance}

  if (exclude_idx != std::numeric_limits<size_t>::max()) {
    matches.erase(std::remove_if(matches.begin(), matches.end(),
      [&](const auto& m){ return m.first == exclude_idx; }), matches.end());
  }
  return matches;
}

std::vector<size_t>
FRRT::getAncestry(const std::vector<Node>& tree,
                  const std::vector<std::pair<size_t,double>>& X_near,
                  int ancestry_depth)
{
  std::vector<size_t> out;
  if (ancestry_depth <= 1 || X_near.empty()) return out;

  const int levels = ancestry_depth - 1;
  out.reserve(X_near.size() * levels);

  std::unordered_set<size_t> seen;
  for (const auto& m : X_near) {
    size_t cur = m.first;
    for (int l = 0; l < levels; ++l) {
      if (cur >= tree.size()) break;
      int p = tree[cur].parent;
      if (p < 0) break;
      size_t ps = static_cast<size_t>(p);
      if (seen.insert(ps).second) out.push_back(ps);
      cur = ps;
    }
  }
  return out;
}

bool FRRT::isDescendant(const std::vector<Node>& tree, size_t root, size_t q) const
{
  if (root >= tree.size() || q >= tree.size()) return false;
  std::deque<size_t> dq; dq.push_back(root);
  while (!dq.empty()) {
    size_t u = dq.front(); dq.pop_front();
    if (u == q) return true;
    for (int ch : tree[u].children) if (ch >= 0) dq.push_back(static_cast<size_t>(ch));
  }
  return false;
}

void FRRT::propagateCost(std::vector<Node>& tree, size_t start, double delta)
{
  std::deque<size_t> dq; dq.push_back(start);
  while (!dq.empty()) {
    size_t u = dq.front(); dq.pop_front();
    tree[u].cost += delta;
    for (int ch : tree[u].children) if (ch >= 0) dq.push_back(static_cast<size_t>(ch));
  }
}

void FRRT::reconnect(std::vector<Node>& tree, size_t new_parent, size_t child)
{
  int old_parent = tree[child].parent;
  if (old_parent >= 0) {
    auto& sib = tree[old_parent].children;
    sib.erase(std::remove(sib.begin(), sib.end(), static_cast<int>(child)), sib.end());
  }
  tree[child].parent = static_cast<int>(new_parent);
  tree[new_parent].children.push_back(static_cast<int>(child));

  // 새 부모로 비용 재계산 + 서브트리 전파
  const double nx = tree[new_parent].pose.pose.position.x;
  const double ny = tree[new_parent].pose.pose.position.y;
  const double cx = tree[child].pose.pose.position.x;
  const double cy = tree[child].pose.pose.position.y;
  const double new_cost = tree[new_parent].cost + std::hypot(cx - nx, cy - ny);
  const double delta    = new_cost - tree[child].cost;
  if (std::abs(delta) > 1e-12) propagateCost(tree, child, delta);
}

// -------------------------- Algorithm 8: FindReachest -------------------------

// x_rand에서 '직접 보이는(LOS)' 최고 조상 반환.
// 어떤 조상도 보이지 않으면 std::numeric_limits<size_t>::max() 반환.
size_t FRRT::findReachest(const std::vector<Node>& tree,
                          size_t x_nearest_idx,
                          const geometry_msgs::msg::PoseStamped& x_rand)
{
  auto los = [&](size_t i){
    const auto& P = tree[i].pose.pose.position;
    return isCollisionFree(x_rand.pose.position.x, x_rand.pose.position.y, P.x, P.y);
  };

  size_t r = x_nearest_idx;
  if (!los(r)) {
    // 위로 올라가며 '보이는' 첫 조상 찾기
    bool found = false;
    while (tree[r].parent >= 0) {
      r = static_cast<size_t>(tree[r].parent);
      if (los(r)) { found = true; break; }
    }
    if (!found) return std::numeric_limits<size_t>::max(); // 아무 조상도 보이지 않음 → 이 샘플 스킵
  }

  // 현재 r은 x_rand에서 보이는 조상.
  // 더 위의 부모도 보이면 계속 올라가 '최고' 조상으로.
  while (tree[r].parent >= 0) {
    size_t p = static_cast<size_t>(tree[r].parent);
    if (los(p)) r = p; else break;
  }
  return r;
}

// --------------------------- Algorithm 9: CreateNode --------------------------

  bool FRRT::createNode(const std::vector<Node>& tree,
                        size_t x_reachest_idx,
                        const geometry_msgs::msg::PoseStamped& x_rand,
                        double D_dichotomy,
                        geometry_msgs::msg::PoseStamped& x_create_out)
  {
    const auto& xr = tree[x_reachest_idx].pose;   // 기준점: x_reachest
    x_create_out = xr;

    // 부모 포즈 (루트면 자기 자신)
    geometry_msgs::msg::PoseStamped parent_pose = xr;
    if (tree[x_reachest_idx].parent >= 0) {
      parent_pose = tree[static_cast<size_t>(tree[x_reachest_idx].parent)].pose;
    }

    // 안전벨트: findReachest가 보장하지만, 혹시라도 깨졌다면 생성 포기
    auto dist2d = [](const auto& a, const auto& b){
      double dx = a.pose.position.x - b.pose.position.x;
      double dy = a.pose.position.y - b.pose.position.y;
      return std::hypot(dx, dy);
    };
    if (!isCollisionFree(xr.pose.position.x, xr.pose.position.y,
                        x_rand.pose.position.x, x_rand.pose.position.y)) {
      return false;
    }

    // [xr, x_rand] 구간에서, parent_pose와의 연결이 안전한 가장 먼 점을 이분탐색
    geometry_msgs::msg::PoseStamped lo = xr;      // 항상 안전(시작점)
    geometry_msgs::msg::PoseStamped hi = x_rand;  // 목표 방향

    auto midpoint = [](const auto& A, const auto& B){
      geometry_msgs::msg::PoseStamped m = A;
      m.pose.position.x = 0.5*(A.pose.position.x + B.pose.position.x);
      m.pose.position.y = 0.5*(A.pose.position.y + B.pose.position.y);
      return m;
    };

    while (dist2d(lo, hi) > D_dichotomy) {
      auto mid = midpoint(lo, hi);
      if (isCollisionFree(mid.pose.position.x, mid.pose.position.y,
                          parent_pose.pose.position.x, parent_pose.pose.position.y)) {
        lo = mid;  // parent와 안전 ⇒ 더 멀리
      } else {
        hi = mid;  // parent와 충돌 ⇒ 가까이
      }
    }

    // lo가 xr에서 의미 있게 전진했으면 채택
    if (dist2d(lo, xr) > 1e-6) {
      x_create_out = lo;
      x_create_out.header = xr.header; // 헤더 유지
      return true;
    }
    return false;
  }

// ------------------------------ Rewire (F-RRT*) -------------------------------

void FRRT::rewireF(std::vector<Node>& tree,
                   size_t x_new_idx,
                   const std::vector<std::pair<size_t,double>>& X_near,
                   const std::vector<size_t>& X_parent)
{
  // X_from = {x_new} ∪ X_parent
  std::vector<size_t> X_from; X_from.reserve(1 + X_parent.size());
  X_from.push_back(x_new_idx);
  std::unordered_set<size_t> seen;
  for (size_t p : X_parent) if (seen.insert(p).second) X_from.push_back(p);

  for (const auto& nm : X_near) {
    const size_t x_near = nm.first;
    const double xn = tree[x_near].pose.pose.position.x;
    const double yn = tree[x_near].pose.pose.position.y;

    for (size_t x_from : X_from) {
      if (x_from == x_near) continue;
      if (isDescendant(tree, x_near, x_from)) continue;   // 사이클 방지

      const double xf = tree[x_from].pose.pose.position.x;
      const double yf = tree[x_from].pose.pose.position.y;

      const double seg_cost = std::hypot(xn - xf, yn - yf);
      const double through  = tree[x_from].cost + seg_cost;

      if (through + 1e-9 < tree[x_near].cost) {
        if (isCollisionFree(xf, yf, xn, yn)) {
          reconnect(tree, x_from, x_near);
        }
      }
    }
  }
}

// ------------------------------- Algorithm 7 ----------------------------------

nav_msgs::msg::Path
FRRT::createPlan(const geometry_msgs::msg::PoseStamped & start_in,
                 const geometry_msgs::msg::PoseStamped & goal_in)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp    = node_->now();

  geometry_msgs::msg::PoseStamped start = start_in;
  geometry_msgs::msg::PoseStamped goal  = goal_in;
  start.header.frame_id = "map";
  goal.header.frame_id  = "map";
  start.header.stamp = path.header.stamp;
  goal.header.stamp  = path.header.stamp;

  std::vector<Node> tree;
  PointCloud cloud;

  // 루트 노드
  Node root;
  root.pose   = start;
  root.pose.pose.orientation.w = 1.0;
  root.parent = -1;
  root.cost   = 0.0;
  tree.push_back(root);
  cloud.pts.push_back({ start.pose.position.x, start.pose.position.y });

  KDTree kdtree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree.buildIndex();

  // 샘플 영역
  std::uniform_real_distribution<> rand_x(
      costmap_ros_->getCostmap()->getOriginX(),
      costmap_ros_->getCostmap()->getOriginX() + costmap_ros_->getCostmap()->getSizeInMetersX());
  std::uniform_real_distribution<> rand_y(
      costmap_ros_->getCostmap()->getOriginY(),
      costmap_ros_->getCostmap()->getOriginY() + costmap_ros_->getCostmap()->getSizeInMetersY());
  std::uniform_real_distribution<> rand_goal_sampling(0.0, 1.0);

  // 파라미터
  const int    MAX_ITERATIONS      = 5000000;
  const double GOAL_BIAS           = 0.1;
  const double GOAL_THRESHOLD      = 1.0;
  const double NEIGHBORHOOD_RADIUS = 30.0;   // R_near
  const int    ANCESTRY_DEPTH      = 2;
  const double D_DICHOTOMY         = 2.0;  // 이분탐색 종료 거리

  const auto t0 = std::chrono::steady_clock::now();

  for (int i = 0; i < MAX_ITERATIONS; ++i) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 60.0) break;

    // 1) SampleFree (goal bias)
    geometry_msgs::msg::PoseStamped x_rand;
    if (rand_goal_sampling(gen) < GOAL_BIAS) x_rand = goal;
    else {
      x_rand.pose.position.x = rand_x(gen);
      x_rand.pose.position.y = rand_y(gen);
      x_rand.header.frame_id = "map";
      x_rand.header.stamp    = path.header.stamp;
    }
    x_rand.pose.orientation.w = 1.0;

    // 2) Nearest
    double query_pt[2] = { x_rand.pose.position.x, x_rand.pose.position.y };
    size_t x_nearest; double out_dist_sqr;
    nanoflann::KNNResultSet<double> rs(1);
    rs.init(&x_nearest, &out_dist_sqr);
    kdtree.findNeighbors(rs, query_pt, nanoflann::SearchParams(10));

    // 3) 충돌 검사: x_rand ↔ x_nearest
    if (!isCollisionFree(x_rand.pose.position.x, x_rand.pose.position.y,
                         tree[x_nearest].pose.pose.position.x,
                         tree[x_nearest].pose.pose.position.y)) {
      continue;
    }

    // 4) X_near
    auto X_near = getNear(kdtree, x_rand.pose.position.x, x_rand.pose.position.y,
                          NEIGHBORHOOD_RADIUS);

    // 5) x_reachest
    size_t x_reachest = findReachest(tree, x_nearest, x_rand);

    // 6) x_create
    geometry_msgs::msg::PoseStamped x_create;
    bool created = createNode(tree, x_reachest, x_rand, D_DICHOTOMY, x_create);

    // 7) 그래프 갱신
    size_t parent_for_rand = x_reachest;
    size_t idx_create = std::numeric_limits<size_t>::max();

    if (created) {
      Node n_create;
      n_create.pose   = x_create;
      n_create.pose.pose.orientation.w = 1.0;
      n_create.parent = static_cast<int>(x_reachest);
      n_create.cost   = tree[x_reachest].cost +
                        std::hypot(x_create.pose.position.x - tree[x_reachest].pose.pose.position.x,
                                   x_create.pose.position.y - tree[x_reachest].pose.pose.position.y);
      tree.push_back(n_create);
      cloud.pts.push_back({ x_create.pose.position.x, x_create.pose.position.y });
      idx_create = tree.size()-1;
      tree[x_reachest].children.push_back(static_cast<int>(idx_create));
      parent_for_rand = idx_create;
    }

    Node n_rand;
    n_rand.pose   = x_rand;
    n_rand.pose.pose.orientation.w = 1.0;
    n_rand.parent = static_cast<int>(parent_for_rand);
    n_rand.cost   = tree[parent_for_rand].cost +
                    std::hypot(x_rand.pose.position.x - tree[parent_for_rand].pose.pose.position.x,
                               x_rand.pose.position.y - tree[parent_for_rand].pose.pose.position.y);
    tree.push_back(n_rand);
    cloud.pts.push_back({ x_rand.pose.position.x, x_rand.pose.position.y });

    const size_t x_rand_idx = tree.size()-1;
    tree[parent_for_rand].children.push_back(static_cast<int>(x_rand_idx));

    // KD-트리 갱신
    kdtree.buildIndex();

    // 8) 초기 경로 발견 검사
    if (std::hypot(goal.pose.position.x - x_rand.pose.position.x,
                   goal.pose.position.y - x_rand.pose.position.y) < GOAL_THRESHOLD)
    {
      // 경로 구성
      for (int idx = static_cast<int>(x_rand_idx); idx != -1; idx = tree[idx].parent)
        path.poses.insert(path.poses.begin(), tree[idx].pose);

      // 헤더/쿼터니언 보정
      for (auto& p : path.poses) {
        p.header.frame_id = path.header.frame_id;  // "map"
        p.header.stamp    = path.header.stamp;
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = 0.0;
        p.pose.orientation.w = 1.0;
      }

      // 경로 고밀도화(0.1m)
      auto densify = [&](nav_msgs::msg::Path& inout, double step){
        nav_msgs::msg::Path out; out.header = inout.header;
        if (inout.poses.empty()) { inout = out; return; }
        out.poses.push_back(inout.poses.front());
        for (size_t i=1; i<inout.poses.size(); ++i) {
          const auto& A = out.poses.back().pose.position;
          const auto& B = inout.poses[i].pose.position;
          double dx=B.x-A.x, dy=B.y-A.y, d=std::hypot(dx,dy);
          if (d < 1e-6) continue;
          int n = std::max(1, (int)std::floor(d/step));
          for (int k=1; k<=n; ++k) {
            geometry_msgs::msg::PoseStamped p = out.poses.back();
            double t = std::min(1.0, k*step/d);
            p.pose.position.x = A.x + dx*t;
            p.pose.position.y = A.y + dy*t;
            p.header = out.header;
            p.pose.orientation.w = 1.0;
            out.poses.push_back(p);
          }
        }
        inout = std::move(out);
      };
      densify(path, 0.1);
      const double duration_us =
          std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(logger_, "[F-RRT*] Path found in %.5f us", duration_us);

// === 경로 길이[m] & 누적 회전각[deg] 계산/출력 ===
      auto compute_path_len_and_turn = [](const nav_msgs::msg::Path& p){
        struct { double len; double turn_rad; } m{0.0, 0.0};
        if (p.poses.size() < 2) return m;

        const double EPS = 1e-6;
        const double PI  = 3.14159265358979323846;

        bool has_prev = false;
        double prev_yaw = 0.0;

        for (size_t i = 1; i < p.poses.size(); ++i) {
          const auto& A = p.poses[i-1].pose.position;
          const auto& B = p.poses[i].pose.position;
          const double dx = B.x - A.x;
          const double dy = B.y - A.y;
          const double seg = std::hypot(dx, dy);
          if (seg < EPS) continue;                 // 0 길이/잡음 무시

          m.len += seg;                            // 총 길이 누적
          const double yaw = std::atan2(dy, dx);   // 현재 선분 진행방향

          if (has_prev) {
            double d = yaw - prev_yaw;             // 방향 변화
            if (d >  PI) d -= 2*PI;                // [-pi, pi] 정규화
            if (d < -PI) d += 2*PI;
            m.turn_rad += std::abs(d);             // 절대값 합 = 누적 회전량
          } else {
            has_prev = true;                       // 첫 선분은 기준만 설정
          }
          prev_yaw = yaw;
        }
        return m;
      };

      const auto M = compute_path_len_and_turn(path);
      const double turn_deg = M.turn_rad * 180.0 / 3.14159265358979323846;
      RCLCPP_INFO(logger_, "[F-RRT*] path length = %.3f m, turn = %.1f deg", M.len, turn_deg);

        
      return path;
    }

    // 9) Rewire
    auto X_parent = getAncestry(tree, X_near, ANCESTRY_DEPTH);
    rewireF(tree, x_rand_idx, X_near, X_parent);
  }

  // 실패 시 빈 path
  return path;
}

} // namespace frrt

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(frrt::FRRT, nav2_core::GlobalPlanner)

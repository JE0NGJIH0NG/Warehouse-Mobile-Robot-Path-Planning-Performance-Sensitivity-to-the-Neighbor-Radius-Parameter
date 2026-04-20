#include "qrrt/qrrt.hpp"

#include <random>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <deque>
#include <limits>
#include <chrono>

#include <nav2_costmap_2d/cost_values.hpp>  // 장애물 임계값 상수

namespace qrrt
{

void QRRT::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string /*name*/,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  RCLCPP_INFO(logger_, "[Q-RRT] Q-RRT Algorithm is configured");
}

void QRRT::cleanup()    { RCLCPP_INFO(logger_, "[Q-RRT] Q-RRT is cleanup."); }
void QRRT::activate()   { RCLCPP_INFO(logger_, "[Q-RRT] Q-RRT is activated"); }
void QRRT::deactivate() { RCLCPP_INFO(logger_, "[Q-RRT] Q-RRT is deactivated"); }

bool QRRT::isCollisionFree(double x0, double y0, double x1, double y1)
{
  unsigned int mx, my;
  const double dist  = std::hypot(x1 - x0, y1 - y0);
  const int    steps = std::max(1, static_cast<int>(dist / 0.05)); // 0 나눗셈 방지

  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double x = x0 + (x1 - x0) * t;
    const double y = y0 + (y1 - y0) * t;

    if (!costmap_ros_->getCostmap()->worldToMap(x, y, mx, my)) {
      return false;
    }
    const unsigned char c = costmap_ros_->getCostmap()->getCost(mx, my);
    if (c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) { // Nav2와 동일 기준
      return false;
    }
  }
  return true;
}

std::vector<std::pair<size_t,double>>
QRRT::getNear(KDTree& kdtree, double x, double y, double r,
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
      [&](const auto& m){ return m.first == exclude_idx; }),
      matches.end());
  }
  return matches;
}

std::vector<size_t>
QRRT::getAncestry(const std::vector<Node>& tree,
                  const std::vector<std::pair<size_t,double>>& X_near,
                  int ancestry_depth)
{
  std::vector<size_t> out;
  if (ancestry_depth <= 1 || X_near.empty()) return out;

  const int levels = ancestry_depth - 1;
  out.reserve(X_near.size() * levels);

  std::unordered_set<size_t> seen;  // 중복 방지
  for (const auto& m : X_near) {
    size_t cur = m.first;           // 이웃 노드 인덱스
    for (int l = 0; l < levels; ++l) {
      if (cur >= tree.size()) break;
      int p = tree[cur].parent;
      if (p < 0) break;             // 루트/없음
      size_t ps = static_cast<size_t>(p);
      if (seen.insert(ps).second) out.push_back(ps);
      cur = ps;                     // 한 단계 위로
    }
  }
  return out;
}

bool QRRT::isDescendant(const std::vector<Node>& tree, size_t ancestor, size_t q) const
{
  if (ancestor >= tree.size() || q >= tree.size()) return false;
  size_t cur = q;
  while (true) {
    if (cur == ancestor) return true;
    if (cur >= tree.size()) return false;
    int p = tree[cur].parent;
    if (p < 0) return false;
    cur = static_cast<size_t>(p);
  }
}

void QRRT::propagateCost(std::vector<Node>& tree, size_t start, double delta)
{
  std::deque<size_t> dq; dq.push_back(start);
  while (!dq.empty()) {
    size_t u = dq.front(); dq.pop_front();
    tree[u].cost += delta;
    for (int ch : tree[u].children) if (ch >= 0) dq.push_back(static_cast<size_t>(ch));
  }
}

void QRRT::reconnect(std::vector<Node>& tree, size_t new_parent, size_t child)
{
  int old_parent = tree[child].parent;
  if (old_parent >= 0) {
    auto& sib = tree[old_parent].children;
    sib.erase(std::remove(sib.begin(), sib.end(), static_cast<int>(child)), sib.end());
  }
  tree[child].parent = static_cast<int>(new_parent);
  tree[new_parent].children.push_back(static_cast<int>(child));

  // 새 부모 경로로 비용 재계산 및 서브트리 전파
  const double nx = tree[new_parent].pose.pose.position.x;
  const double ny = tree[new_parent].pose.pose.position.y;
  const double cx = tree[child].pose.pose.position.x;
  const double cy = tree[child].pose.pose.position.y;
  const double new_cost = tree[new_parent].cost + std::hypot(cx - nx, cy - ny);
  const double delta    = new_cost - tree[child].cost;
  if (std::abs(delta) > 1e-12) propagateCost(tree, child, delta);
}

void QRRT::rewireQ(std::vector<Node>& tree,
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

nav_msgs::msg::Path
QRRT::createPlan(const geometry_msgs::msg::PoseStamped & start_in,
                 const geometry_msgs::msg::PoseStamped & goal_in)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp    = node_->now();

  // 입력 포즈 보정(프레임/스탬프)
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
  root.pose  = start;
  root.parent = -1;
  root.cost   = 0.0;
  tree.push_back(root);
  cloud.pts.push_back({ start.pose.position.x, start.pose.position.y });

  KDTree kdtree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  kdtree.buildIndex();

  // 샘플 영역: 현재 costmap 전체
  std::uniform_real_distribution<> rand_x(
      costmap_ros_->getCostmap()->getOriginX(),
      costmap_ros_->getCostmap()->getOriginX() + costmap_ros_->getCostmap()->getSizeInMetersX());
  std::uniform_real_distribution<> rand_y(
      costmap_ros_->getCostmap()->getOriginY(),
      costmap_ros_->getCostmap()->getOriginY() + costmap_ros_->getCostmap()->getSizeInMetersY());
  std::uniform_real_distribution<> rand_goal_sampling(0.0, 1.0);

  const int    MAX_ITERATIONS      = 5000000;
  const double STEP_SIZE           = 1.0;
  const double GOAL_THRESHOLD      = 1.0;
  const double GOAL_BIAS           = 0.1;
  const double NEIGHBORHOOD_RADIUS = 30.0;
  const int    ANCESTRY_DEPTH      = 2;

  const auto t0 = std::chrono::steady_clock::now();

  for (int i = 0; i < MAX_ITERATIONS; ++i) {
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 60.0) break;

    // 1) 샘플링 (goal bias)
    geometry_msgs::msg::PoseStamped q_rand;
    if (rand_goal_sampling(gen) < GOAL_BIAS) q_rand = goal;
    else {
      q_rand.pose.position.x = rand_x(gen);
      q_rand.pose.position.y = rand_y(gen);
      q_rand.header.frame_id = "map";
      q_rand.header.stamp    = path.header.stamp;
    }

    // 2) 최근접
    double query_pt[2] = { q_rand.pose.position.x, q_rand.pose.position.y };
    size_t nearest_idx;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&nearest_idx, &out_dist_sqr);
    kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

    // 3) Steer
    const double angle = std::atan2(
        q_rand.pose.position.y - tree[nearest_idx].pose.pose.position.y,
        q_rand.pose.position.x - tree[nearest_idx].pose.pose.position.x);
    const double new_x = tree[nearest_idx].pose.pose.position.x + STEP_SIZE * std::cos(angle);
    const double new_y = tree[nearest_idx].pose.pose.position.y + STEP_SIZE * std::sin(angle);

    if (!isCollisionFree(tree[nearest_idx].pose.pose.position.x,
                         tree[nearest_idx].pose.pose.position.y,
                         new_x, new_y)) continue;

    // 4) 근방/조상
    auto X_near   = getNear(kdtree, new_x, new_y, NEIGHBORHOOD_RADIUS);
    auto X_parent = getAncestry(tree, X_near, ANCESTRY_DEPTH);

    // 5) 부모 선택: {nearest} ∪ X_near ∪ X_parent
    std::vector<size_t> candidates;
    candidates.reserve(1 + X_near.size() + X_parent.size());
    std::unordered_set<size_t> seen;
    auto add_unique = [&](size_t i){ if (seen.insert(i).second) candidates.push_back(i); };
    add_unique(nearest_idx);
    for (auto& m : X_near)   add_unique(m.first);
    for (auto& i2: X_parent) add_unique(i2);

    size_t best_parent_idx = nearest_idx;
    double min_cost = tree[nearest_idx].cost +
                      std::hypot(new_x - tree[nearest_idx].pose.pose.position.x,
                                 new_y - tree[nearest_idx].pose.pose.position.y);

    for (size_t idx : candidates) {
      const double nx = tree[idx].pose.pose.position.x;
      const double ny = tree[idx].pose.pose.position.y;
      if (!isCollisionFree(nx, ny, new_x, new_y)) continue;

      const double new_cost = tree[idx].cost + std::hypot(nx - new_x, ny - new_y);
      if (new_cost < min_cost) { min_cost = new_cost; best_parent_idx = idx; }
    }

    // 6) Connect (q_new)
    Node q_new;
    q_new.pose.header.frame_id = "map";
    q_new.pose.header.stamp    = path.header.stamp;
    q_new.pose.pose.position.x = new_x;
    q_new.pose.pose.position.y = new_y;
    q_new.parent = static_cast<int>(best_parent_idx);
    q_new.cost   = min_cost;  // ← 고정 STEP_SIZE가 아니라 최적 비용

    tree.push_back(q_new);
    cloud.pts.push_back({new_x, new_y});
    const int q_new_idx = static_cast<int>(tree.size()) - 1;
    tree[best_parent_idx].children.push_back(q_new_idx);

    // 새 점 추가 → KD-트리 갱신 (rewire는 좌표 안 바꾸므로 이후 갱신 불필요)
    kdtree.buildIndex();

    // 7) Rewire-Q-RRT*
    rewireQ(tree, static_cast<size_t>(q_new_idx), X_near, X_parent);

    // 8) Goal 체크
    if (std::hypot(goal.pose.position.x - new_x,
                   goal.pose.position.y - new_y) < GOAL_THRESHOLD)
    {
      const double duration_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(logger_, "[Q-RRT*] Path found in %.5f us", duration_us);
      // 경로 재구성
      for (int idx = q_new_idx; idx != -1; idx = tree[idx].parent)
        path.poses.insert(path.poses.begin(), tree[idx].pose);

      // 경로 헤더/포즈 헤더 보정
      for (auto& p : path.poses) {
        p.header.frame_id = path.header.frame_id;  // "map"
        p.header.stamp    = path.header.stamp;
      }

      // 경로 고밀도화(0.1m 간격) → DWB 안정성 향상
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
            p.header.frame_id = out.header.frame_id;
            p.header.stamp    = out.header.stamp;
            out.poses.push_back(p);
          }
        }
        inout = std::move(out);
      };
      densify(path, 0.1);

      RCLCPP_INFO(logger_, "[Q-RRT*] plan size=%zu  first=(%.2f,%.2f)  next=(%.2f,%.2f)",
                  path.poses.size(),
                  path.poses.size()?path.poses.front().pose.position.x:0.0,
                  path.poses.size()?path.poses.front().pose.position.y:0.0,
                  path.poses.size()>1?path.poses[1].pose.position.x:0.0,
                  path.poses.size()>1?path.poses[1].pose.position.y:0.0);


      // === 추가: 경로 길이[m]와 누적 회전각[deg] 계산/출력 ===
      auto compute_path_len_and_turn = [](const nav_msgs::msg::Path& p){
        struct { double len; double turn_rad; } m{0.0, 0.0};
        const double EPS = 1e-6;
        const double PI  = 3.14159265358979323846;

        if (p.poses.size() < 2) return m;

        bool has_prev_yaw = false;
        double prev_yaw = 0.0;

        for (size_t i = 1; i < p.poses.size(); ++i) {
          const auto& A = p.poses[i-1].pose.position;
          const auto& B = p.poses[i].pose.position;
          const double dx = B.x - A.x;
          const double dy = B.y - A.y;
          const double seg = std::hypot(dx, dy);
          if (seg < EPS) continue;               // 중복점/잡음 스킵

          m.len += seg;                           // 총 길이 누적
          const double yaw = std::atan2(dy, dx);  // 현재 선분의 진행 방향(라디안)

          if (has_prev_yaw) {
            double d = yaw - prev_yaw;           // 방향 변화량
            if (d >  PI) d -= 2*PI;              // [-pi, pi] 정규화
            if (d < -PI) d += 2*PI;
            m.turn_rad += std::abs(d);           // 절대값 합 = 누적 회전량
          } else {
            has_prev_yaw = true;
          }
          prev_yaw = yaw;
        }
        return m;
      };

      const auto M = compute_path_len_and_turn(path);
      const double turn_deg = M.turn_rad * 180.0 / 3.14159265358979323846;
      RCLCPP_INFO(logger_, "[Q-RRT*] path length = %.3f m, turn = %.1f deg", M.len, turn_deg);

      return path;
    }
  }

  // 실패시 빈 경로 반환
  return path;
}

} // namespace qrrt

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(qrrt::QRRT, nav2_core::GlobalPlanner)

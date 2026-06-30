#ifndef FRRT__FRRT_HPP_
#define FRRT__FRRT_HPP_

#include <nav2_core/global_planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

#include <nanoflann.hpp>
#include <vector>
#include <utility>
#include <limits>

namespace frrt
{

class FRRT : public nav2_core::GlobalPlanner
{
public:
  FRRT() = default;
  ~FRRT() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                 const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // ---------- ROS ----------
  rclcpp::Logger logger_{rclcpp::get_logger("F-RRT* Algorithm")};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // ---------- Data types (QRRT와 동일 스타일) ----------
  struct PointCloud {
    struct Point { double x, y; };
    std::vector<Point> pts;
    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_get_pt(const size_t idx, int dim) const {
      return dim == 0 ? pts[idx].x : pts[idx].y;
    }
    template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
  };

  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, PointCloud>,
      PointCloud, 2>;

  struct Node {
    geometry_msgs::msg::PoseStamped pose;
    int    parent = -1;
    double cost   = 0.0;
    std::vector<int> children;
  };

  // ---------- Core utils ----------
  bool isCollisionFree(double x0, double y0, double x1, double y1);

  std::vector<std::pair<size_t,double>>
  getNear(KDTree& kdtree, double x, double y, double r,
          size_t exclude_idx = std::numeric_limits<size_t>::max(),
          bool sort_by_distance = false);

  // X_near의 (ANCESTRY_DEPTH-1) 레벨 조상 집합
  std::vector<size_t>
  getAncestry(const std::vector<Node>& tree,
              const std::vector<std::pair<size_t,double>>& X_near,
              int ancestry_depth);

  // ---------- F-RRT* 전용: FindReachest / CreateNode ----------
  // Algorithm 8: x_nearest에서 루트 쪽으로 가능한 한 멀리(=reachest) 올라감
  size_t findReachest(const std::vector<Node>& tree,
                      size_t x_nearest_idx,
                      const geometry_msgs::msg::PoseStamped& x_rand);

  // Algorithm 9: dichotomy로 x_create 생성 (성공 시 true, 실패 시 false)
  bool createNode(const std::vector<Node>& tree,
                  size_t x_reachest_idx,
                  const geometry_msgs::msg::PoseStamped& x_rand,
                  double D_dichotomy,
                  geometry_msgs::msg::PoseStamped& x_create_out);

  // ---------- Rewire (Q-RRT*와 동일 패턴, x_from={x_new}∪ancestor) ----------
  void rewireF(std::vector<Node>& tree,
               size_t x_new_idx,
               const std::vector<std::pair<size_t,double>>& X_near,
               const std::vector<size_t>& X_parent);

  // 내부 유틸 (부모 변경/비용 전파/사이클 방지)
  void reconnect(std::vector<Node>& tree, size_t new_parent, size_t child);
  void propagateCost(std::vector<Node>& tree, size_t start, double delta);
  bool isDescendant(const std::vector<Node>& tree, size_t root, size_t q) const;

  // 보조: 거리/중간점
  static inline double distance2d(const geometry_msgs::msg::PoseStamped& a,
                                  const geometry_msgs::msg::PoseStamped& b) {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }
  static inline geometry_msgs::msg::PoseStamped
  midpoint(const geometry_msgs::msg::PoseStamped& a,
           const geometry_msgs::msg::PoseStamped& b) {
    geometry_msgs::msg::PoseStamped m = a;
    m.pose.position.x = 0.5*(a.pose.position.x + b.pose.position.x);
    m.pose.position.y = 0.5*(a.pose.position.y + b.pose.position.y);
    return m;
  }
};

}  // namespace frrt

#endif  // FRRT__FRRT_HPP_

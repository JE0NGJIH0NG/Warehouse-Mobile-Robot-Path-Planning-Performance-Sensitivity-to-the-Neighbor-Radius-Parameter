#ifndef QRRT__QRRT_HPP_
#define QRRT__QRRT_HPP_

#include <nav2_core/global_planner.hpp>   //nav2의 global planner interface 선언
#include <geometry_msgs/msg/pose_stamped.hpp> // start와 goal의 pose를 나타내는 메시지 타입
#include <nav_msgs/msg/path.hpp> // path를 담는 메시지 타입
#include <rclcpp/rclcpp.hpp>  // ros2 c++ 라이브러리
#include <tf2_ros/buffer.h>     // tf transform을 조회하기 위한 버퍼 클래스 
#include <nav2_costmap_2d/costmap_2d_ros.hpp> // nav2의 costmap2D ROS wrapper

#include <nanoflann.hpp>// kd tree
#include <vector> // dynamic array
#include <utility> // util
#include <limits> // min max 상수

namespace qrrt {// q rrt 네임스페이스 시작점

// :은 상속을 뜻한다. ::은 범위 지정 연산자.
class QRRT : public nav2_core::GlobalPlanner // nav2_core의 global planner을 상속받는 qrrt 클래스를 구현할게용 
{
public:
  QRRT() = default; // 생성자 
  ~QRRT() override = default; // 소멸자 

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
  rclcpp::Logger logger_{rclcpp::get_logger("QRRT Algorithm")};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

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

  bool isCollisionFree(double x0, double y0, double x1, double y1);

  std::vector<std::pair<size_t,double>>
  getNear(KDTree& kdtree, double x, double y, double r,
          size_t exclude_idx = std::numeric_limits<size_t>::max(),
          bool sort_by_distance = true);

  std::vector<size_t>
  getAncestry(const std::vector<Node>& tree,
              const std::vector<std::pair<size_t,double>>& X_near,
              int ancestry_depth);

  void rewireQ(std::vector<Node>& tree,
               size_t x_new_idx,
               const std::vector<std::pair<size_t,double>>& X_near,
               const std::vector<size_t>& X_parent);

  void reconnect(std::vector<Node>& tree, size_t new_parent, size_t child);
  void propagateCost(std::vector<Node>& tree, size_t start, double delta);
  bool isDescendant(const std::vector<Node>& tree, size_t root, size_t q) const;
};

}  // namespace qrrt

#endif  // QRRT__QRRT_HPP_

/**
* This file is part of Fast-Planner.
*
* GNU LGPL v3
*/

#include <traj_utils/planning_visualization.h>
#include <std_msgs/ColorRGBA.h>
#include <sstream>

using std::cout;
using std::endl;

namespace fast_planner {

PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh) {
  node = nh;

  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 20);
  pubs_.push_back(traj_pub_);

  topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 20);
  pubs_.push_back(topo_pub_);

  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 20);
  pubs_.push_back(predict_pub_);

  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/visib_constraint", 20);
  pubs_.push_back(visib_pub_);

  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 20);
  pubs_.push_back(frontier_pub_);

  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 20);
  pubs_.push_back(yaw_pub_);

  last_topo_path1_num_     = 0;
  last_topo_path2_num_     = 0;
  last_bspline_phase1_num_ = 0;
  last_bspline_phase2_num_ = 0;
  last_frontier_num_       = 0;
}

// ------------------------ 基础形状 ------------------------
void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                                              const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "camera_init";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < static_cast<int>(list.size()); i++) {
    pt.x = list[i](0);  // 修复：访问向量元素的x坐标
    pt.y = list[i](1);  // 修复：访问向量元素的y坐标
    pt.z = list[i](2);  // 修复：访问向量元素的z坐标
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "camera_init";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::CUBE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < static_cast<int>(list.size()); i++) {
    pt.x = list[i](0);  // 修复：访问向量元素的x坐标
    pt.y = list[i](1);  // 修复：访问向量元素的y坐标
    pt.z = list[i](2);  // 修复：访问向量元素的z坐标
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1,
                                            const vector<Eigen::Vector3d>& list2, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "camera_init";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::LINE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  int n = std::min(static_cast<int>(list1.size()), static_cast<int>(list2.size()));
  for (int i = 0; i < n; ++i) {
    pt.x = list1[i](0);  // 修复：访问向量元素的x坐标
    pt.y = list1[i](1);  // 修复：访问向量元素的y坐标
    pt.z = list1[i](2);  // 修复：访问向量元素的z坐标
    mk.points.push_back(pt);

    pt.x = list2[i](0);  // 修复：访问向量元素的x坐标
    pt.y = list2[i](1);  // 修复：访问向量元素的y坐标
    pt.z = list2[i](2);  // 修复：访问向量元素的z坐标
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.001).sleep();
}

void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,
                                              const Eigen::Vector4d& color, int id) {
  if (path.size() < 2) return;

  const int line_id = PATH + id % 100;
  const int text_id = line_id + 1000;

  // 清掉旧线 & 旧文本
  {
    visualization_msgs::Marker del;
    del.header.frame_id = "camera_init";
    del.header.stamp    = ros::Time::now();

    del.type   = visualization_msgs::Marker::LINE_STRIP;
    del.action = visualization_msgs::Marker::DELETE;
    del.id     = line_id;
    pubs_[0].publish(del);

    del.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;
    del.id     = text_id;
    pubs_[0].publish(del);
  }

  // 计算总长度（用于渐变 & 长度标注）
  double total_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) total_length += (path[i] - path[i - 1]).norm();

  // 单条线：逐点渐变+轻微 alpha 脉冲；加粗一点
  visualization_msgs::Marker line;
  line.header.frame_id = "camera_init";
  line.header.stamp    = ros::Time::now();
  line.type            = visualization_msgs::Marker::LINE_STRIP;
  line.action          = visualization_msgs::Marker::ADD;
  line.id              = line_id;
  line.pose.orientation.w = 1.0;
  line.scale.x = std::max(0.1, resolution * 3.0); // <<< 深度加大（线更粗）

  // 渐变端点颜色（青 -> 紫），alpha 取入参或默认偏亮
  // Eigen::Vector4d c_start(0.0, 0.8, 1.0, (color(3) > 0 ? color(3) : 0.99));
  // Eigen::Vector4d c_end  (0.8, 0.0, 1.0, (color(3) > 0 ? color(3) : 0.99));
  Eigen::Vector4d c_start(0.0, 0.8, 1.0, 0.99);
  Eigen::Vector4d c_end  (0.8, 0.0, 1.0, 0.99);
  
  geometry_msgs::Point pt;
  std_msgs::ColorRGBA  cr;

  double accum = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) accum += (path[i] - path[i - 1]).norm();
    double t = (total_length > 1e-9) ? (accum / total_length)
                                     : (double)i / std::max<size_t>(1, path.size() - 1);

    // 颜色插值 + 轻微 alpha 脉冲（单线也有“发光”的感觉）
    Eigen::Vector4d g = (1.0 - t) * c_start + t * c_end;
    double alpha_pulse = 0.6 + 0.4 * sin(t * M_PI); // 0.6~1.0
    g(3) *= alpha_pulse;

    // 保持你的写法风格
    pt.x = path[i](0);  // 修复：访问向量元素的x坐标
    pt.y = path[i](1);  // 修复：访问向量元素的y坐标
    pt.z = path[i](2);  // 修复：访问向量元素的z坐标
    line.points.push_back(pt);

    cr.r = g(0);
    cr.g = g(1);
    cr.b = g(2);
    cr.a = g(3);
    line.colors.push_back(cr);
  }

  // 兜底颜色（有逐点 colors 时 RViz 会优先生效）
  line.color.r = color(0);
  line.color.g = color(1);
  line.color.b = color(2);
  line.color.a = (color(3) > 0 ? color(3) : 0.9);

  pubs_[0].publish(line);

  // —— 在“起点约 1 m 处”的路径旁边标注总长度 ——
  if (total_length > 1e-6) {
    // 目标弧长（若路径短于 1m，则取总长的一半）
    const double s_target = std::min(1.0, std::max(0.0, total_length * 0.5 * (total_length < 1.0)));

    // 找到 s_target 所在段并线性插值
    double s_acc = 0.0;
    Eigen::Vector3d p_mark = path.front();
    Eigen::Vector3d tangent(1, 0, 0);

    for (size_t i = 1; i < path.size(); ++i) {
      Eigen::Vector3d a = path[i - 1];
      Eigen::Vector3d b = path[i];
      double seg = (b - a).norm();

      if (s_acc + seg >= s_target) {
        double r = (seg > 1e-9) ? ((s_target - s_acc) / seg) : 0.0;
        p_mark   = a + r * (b - a);
        tangent  = (b - a).normalized();
        break;
      }
      s_acc += seg;
      if (i + 1 == path.size()) {
        p_mark  = b;
        tangent = (b - a).normalized();
      }
    }

    // 轻微侧向偏移（“旁边”），避免与线重叠
    Eigen::Vector3d up(0, 0, 1);
    if (fabs(tangent.dot(up)) > 0.95) up = Eigen::Vector3d(0, 1, 0);
    Eigen::Vector3d right = tangent.cross(up);
    if (right.norm() < 1e-6) right = Eigen::Vector3d(1, 0, 0);
    right.normalize();

    Eigen::Vector3d p_text = p_mark + 0.18 * right + Eigen::Vector3d(0, 0, 0.08);

    // 文本标注
    visualization_msgs::Marker text;
    text.header.frame_id = "camera_init";
    text.header.stamp    = ros::Time::now();
    text.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action          = visualization_msgs::Marker::ADD;
    text.id              = text_id;
    text.pose.orientation.w = 1.0;

    // 位置
    text.pose.position.x = p_text(0);
    text.pose.position.y = p_text(1);
    text.pose.position.z = p_text(2);

    // 文本内容：“xx.x m”
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(1);
    ss << total_length << " m";
    text.text = ss.str();

    // 字号 & 颜色
    text.scale.z = 0.30; // 字高
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 0.95;

    pubs_[0].publish(text);
  }
}


void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                                               const Eigen::Vector4d& color, int id) {
  poly_traj.init();
  vector<Eigen::Vector3d> poly_pts = poly_traj.getTraj();
  
  // 使用增强的路径绘制方法
  drawGeometricPath(poly_pts, resolution, color, id);
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, int id2) {
  if (bspline.getControlPoint().size() == 0) return;

  // 高密度采样得到超平滑轨迹点
  vector<Eigen::Vector3d> traj_pts;
  double t0, t1;
  bspline.getTimeSpan(t0, t1);
  double dt = std::min(0.005, (t1 - t0) / 400.0); // 超高密度采样
  for (double t = t0; t <= t1 + 1e-6; t += dt) {
    traj_pts.push_back(bspline.evaluateDeBoor(t));
  }

  // 计算轨迹长度和速度信息
  double length = 0.0;
  vector<double> speeds;
  speeds.reserve(traj_pts.size());
  speeds.push_back(0.0);
  
  for (int i = 1; i < static_cast<int>(traj_pts.size()); ++i) {
    double seg_len = (traj_pts[i] - traj_pts[i - 1]).norm();
    length += seg_len;
    speeds.push_back(seg_len / dt); // 近似速度
  }

  // 清除旧的可视化
  const int line_id = BSPLINE + id1 % 100;
  const int glow_id = line_id + 100;
  const int text_id = line_id + 1000;

  {
    visualization_msgs::Marker del;
    del.header.frame_id = "camera_init";
    del.header.stamp    = ros::Time::now();
    del.action          = visualization_msgs::Marker::DELETE;

    del.type = visualization_msgs::Marker::LINE_STRIP;
    del.id   = line_id;
    pubs_[0].publish(del);
    
    del.id = glow_id;
    pubs_[0].publish(del);

    del.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    del.id   = text_id;
    pubs_[0].publish(del);
  }

  // 创建炫酷的发光管状轨迹
  // 1. 外层发光效果（粗线，半透明）
  visualization_msgs::Marker glow_line;
  glow_line.header.frame_id = "camera_init";
  glow_line.header.stamp = ros::Time::now();
  glow_line.type = visualization_msgs::Marker::LINE_STRIP;
  glow_line.action = visualization_msgs::Marker::ADD;
  glow_line.id = glow_id;
  glow_line.pose.orientation.w = 1.0;
  glow_line.scale.x = std::max(0.08, size * 3.0); // 粗的外层
  
  // 外层使用柔和的渐变色（青-蓝-紫）
  glow_line.color.r = 0.2;
  glow_line.color.g = 0.6;
  glow_line.color.b = 1.0;
  glow_line.color.a = 0.4; // 半透明发光效果
  
  geometry_msgs::Point pt;
  for (const auto& p : traj_pts) {
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    glow_line.points.push_back(pt);
  }
  pubs_[0].publish(glow_line);
  
  // 2. 内层主轨迹（中等粗细，鲜艳颜色）
  visualization_msgs::Marker main_line;
  main_line.header.frame_id = "camera_init";
  main_line.header.stamp = ros::Time::now();
  main_line.type = visualization_msgs::Marker::LINE_STRIP;
  main_line.action = visualization_msgs::Marker::ADD;
  main_line.id = line_id;
  main_line.pose.orientation.w = 1.0;
  main_line.scale.x = std::max(0.04, size * 1.5); // 中等粗细
  
  // 使用科技感的渐变色：从电蓝色到荧光绿
  main_line.color.r = 0.0;
  main_line.color.g = 0.9;
  main_line.color.b = 1.0;
  main_line.color.a = 0.95;
  
  for (const auto& p : traj_pts) {
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    main_line.points.push_back(pt);
  }
  pubs_[0].publish(main_line);

  // 可选：绘制控制点
  if (show_ctrl_pts) {
    Eigen::MatrixXd ctrl = bspline.getControlPoint();
    vector<Eigen::Vector3d> ctp;
    ctp.reserve(ctrl.rows());
    for (int i = 0; i < ctrl.rows(); ++i) {
      ctp.emplace_back(ctrl(i,0), ctrl(i,1), ctrl(i,2));
    }
    
    // 控制点使用金色小球
    Eigen::Vector4d ctrl_color(1.0, 0.8, 0.0, 0.8);
    displaySphereList(ctp, size2, ctrl_color, BSPLINE_CTRL_PT + id2 % 100, 0);
  } else {
    // 清除控制点
    visualization_msgs::Marker del;
    del.header.frame_id = "camera_init";
    del.header.stamp    = ros::Time::now();
    del.type            = visualization_msgs::Marker::SPHERE_LIST;
    del.action          = visualization_msgs::Marker::DELETE;
    del.id              = BSPLINE_CTRL_PT + id2 % 100;
    pubs_[0].publish(del);
  }

  // 简洁的长度信息显示
  if (!traj_pts.empty()) {
    visualization_msgs::Marker text;
    text.header.frame_id = "camera_init";
    text.header.stamp    = ros::Time::now();
    text.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action          = visualization_msgs::Marker::ADD;
    text.id              = text_id;

    const auto& last = traj_pts.back();
    text.pose.position.x = last(0);
    text.pose.position.y = last(1);
    text.pose.position.z = last(2) + 0.3;
    text.pose.orientation.w = 1.0;

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(1);
    ss << length << "m";  // 简洁显示
    text.text = ss.str();

    text.scale.z = 0.25; // 较小的字体
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 0.8;

    pubs_[0].publish(text);
  }

  ros::Duration(0.001).sleep();
}

void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline>& bsplines, double size) {
  // 清空上一批 Phase1
  vector<Eigen::Vector3d> empty;
  for (int i = 0; i < last_bspline_phase1_num_; ++i) {
    // 清线
    visualization_msgs::Marker del;
    del.header.frame_id = "camera_init";
    del.header.stamp    = ros::Time::now();
    del.action          = visualization_msgs::Marker::DELETE;

    del.type = visualization_msgs::Marker::LINE_STRIP;
    del.id   = BSPLINE + i % 100;
    pubs_[0].publish(del);

    // 清控制点
    del.type = visualization_msgs::Marker::SPHERE_LIST;
    del.id   = BSPLINE_CTRL_PT + i % 100;
    pubs_[0].publish(del);
  }
  last_bspline_phase1_num_ = bsplines.size();

  // 绘制新的（增强可视化效果）
  const int N = static_cast<int>(bsplines.size());
  for (int i = 0; i < N; ++i) {
    // 使用更丰富的色彩渐变
    Eigen::Vector4d col = getColor((N > 1) ? double(i) / (N - 1) : 0.0, 0.9);
    col(3) = 0.8; // 设置合适的透明度
    drawBspline(bsplines[i], std::max(0.03, size), col, false, 2 * size,
                getColor((N > 1) ? double(i) / (N - 1) : 0.0, 0.6), i, i);
  }
}

void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline>& bsplines, double size) {
  // 清空上一批 Phase2
  for (int i = 0; i < last_bspline_phase2_num_; ++i) {
    visualization_msgs::Marker del;
    del.header.frame_id = "camera_init";
    del.header.stamp    = ros::Time::now();
    del.action          = visualization_msgs::Marker::DELETE;

    del.type = visualization_msgs::Marker::LINE_STRIP;
    del.id   = BSPLINE + (50 + i) % 100;
    pubs_[0].publish(del);

    del.type = visualization_msgs::Marker::SPHERE_LIST;
    del.id   = BSPLINE_CTRL_PT + (50 + i) % 100;
    pubs_[0].publish(del);
  }
  last_bspline_phase2_num_ = bsplines.size();

  const int N = static_cast<int>(bsplines.size());
  for (int i = 0; i < N; ++i) {
    // Phase2使用不同的色彩方案，更醒目
    Eigen::Vector4d col = getColor((N > 1) ? double(i) / (N - 1) : 0.0, 1.0);
    col(3) = 0.95; // 更高的不透明度
    drawBspline(bsplines[i], std::max(0.035, size), col, false, 1.8 * size,
                getColor((N > 1) ? double(i) / (N - 1) : 0.0, 0.7), 50 + i, 50 + i);
  }
}

// ------------------------ 拓扑图/路径 ------------------------
void PlanningVisualization::drawTopoGraph(std::list<GraphNode::Ptr>& graph, double point_size,
                                          double line_width, const Eigen::Vector4d& color1,
                                          const Eigen::Vector4d& color2, const Eigen::Vector4d& color3,
                                          int id) {
  // clear existing node and edge (drawn last time)
  vector<Eigen::Vector3d> empty;
  displaySphereList(empty, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(empty, point_size, color1, GRAPH_NODE + 50, 1);
  displayLineList(empty, empty, line_width, color3, GRAPH_EDGE, 1);

  /* draw graph node */
  vector<Eigen::Vector3d> guards, connectors;
  for (auto& n : graph) {
    if (n->type_ == GraphNode::Guard)
      guards.push_back(n->pos_);
    else if (n->type_ == GraphNode::Connector)
      connectors.push_back(n->pos_);
  }
  displaySphereList(guards, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(connectors, point_size, color2, GRAPH_NODE + 50, 1);

  /* draw graph edge */
  vector<Eigen::Vector3d> edge_pt1, edge_pt2;
  for (auto& n : graph) {
    for (int k = 0; k < static_cast<int>(n->neighbors_.size()); ++k) {
      edge_pt1.push_back(n->pos_);
      edge_pt2.push_back(n->neighbors_[k]->pos_);
    }
  }
  displayLineList(edge_pt1, edge_pt2, line_width, color3, GRAPH_EDGE, 1);
}

void PlanningVisualization::drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>>& paths,
                                                double                           line_width) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path1_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, line_width, color1, SELECT_PATH + i % 100, 1);
    displaySphereList(empty, line_width, color1, PATH + i % 100, 1);
  }
  last_topo_path1_num_ = paths.size();

  // draw new paths with enhanced visualization
  for (int i = 0; i < static_cast<int>(paths.size()); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;
    for (int j = 0; j < static_cast<int>(paths[i].size()) - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }
    Eigen::Vector4d path_color = getColor(double(i) / std::max(1,int(paths.size()-1)), 0.8);
    displayLineList(edge_pt1, edge_pt2, std::max(0.02, line_width), path_color,
                    SELECT_PATH + i % 100, 1);
  }
}

void PlanningVisualization::drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>>& paths, double size) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path2_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, size, color1, FILTERED_PATH + i % 100, 1);
  }
  last_topo_path2_num_ = paths.size();

  // draw new paths with enhanced visualization
  for (int i = 0; i < static_cast<int>(paths.size()); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;
    for (int j = 0; j < static_cast<int>(paths[i].size()) - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }
    Eigen::Vector4d path_color = getColor(double(i) / std::max(1,int(paths.size()-1)), 0.85);
    displayLineList(edge_pt1, edge_pt2, std::max(0.025, size), path_color,
                    FILTERED_PATH + i % 100, 1);
  }
}

// ------------------------ 目标/预测/Yaw ------------------------
void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  
  // 增强目标点可视化：添加一个较大的外圈和脉冲效果
  const int goal_id = GOAL + id % 100;
  const int halo_id = goal_id + 300;
  
  // 主目标点
  displaySphereList(goal_vec, resolution, color, goal_id, 0);
  
  // 外圈光晕效果
  Eigen::Vector4d halo_color = color;
  halo_color(3) = 0.3; // 半透明
  displaySphereList(goal_vec, resolution * 2.5, halo_color, halo_id, 0);
}

void PlanningVisualization::drawPrediction(ObjPrediction pred, double resolution,
                                           const Eigen::Vector4d& color, int id) {
  ros::Time    time_now   = ros::Time::now();
  double       start_time = (time_now - ObjHistory::global_start_time_).toSec();
  const double range      = 5.6;

  vector<Eigen::Vector3d> traj;
  for (int i = 0; i < static_cast<int>(pred->size()); i++) {
    PolynomialPrediction poly = pred->at(i);
    if (!poly.valid()) continue;

    for (double t = start_time; t <= start_time + range; t += 0.5) { // 密集采样
      Eigen::Vector3d pt = poly.evaluateConstVel(t);
      traj.push_back(pt);
    }
  }
  
  // 使用稍微透明的球体显示预测轨迹
  Eigen::Vector4d pred_color = color;
  pred_color(3) = 0.6; // 透明度表示不确定性
  displaySphereList(traj, resolution * 0.8, pred_color, id % 100, 2);
}

void PlanningVisualization::drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw,
                                        const double& dt) {
  double                  duration = pos.getTimeSum();
  vector<Eigen::Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    pc[2] += 0.15;
    double          yc = yaw.evaluateDeBoorT(tc)[0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.2 * dir; // 稍微长一些的箭头
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.05, Eigen::Vector4d(1, 0.6, 0, 1), 0, 5); // 更粗的yaw线
}

void PlanningVisualization::drawYawPath(NonUniformBspline& pos, const vector<double>& yaw,
                                        const double& dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < static_cast<int>(yaw.size()); ++i) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 1.2 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 1), 1, 5);
}

// ------------------------ 颜色工具 ------------------------
Eigen::Vector4d PlanningVisualization::getColor(double h, double alpha) {
  if (h < 0.0 || h > 1.0) {
    std::cout << "h out of range" << std::endl;
    h = 0.0;
  }

  double          lambda = 0.0;
  Eigen::Vector4d color1(1,0,0,1), color2(1,0,1,1); // init

  if (h >= -1e-4 && h < 1.0 / 6) {
    lambda = (h - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);

  } else if (h >= 1.0 / 6 && h < 2.0 / 6) {
    lambda = (h - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);

  } else if (h >= 2.0 / 6 && h < 3.0 / 6) {
    lambda = (h - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);

  } else if (h >= 3.0 / 6 && h < 4.0 / 6) {
    lambda = (h - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);

  } else if (h >= 4.0 / 6 && h < 5.0 / 6) {
    lambda = (h - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);

  } else if (h >= 5.0 / 6 && h <= 1.0 + 1e-4) {
    lambda = (h - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;
  return fcolor;
}

}  // namespace fast_planner
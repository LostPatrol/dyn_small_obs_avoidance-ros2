// GPL-3.0: derived from HKU-MARS 9d25dd6; original search/math retained with documented fixes.
#include <path_searching/kinodynamic_astar.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
using namespace std;
using namespace Eigen;

void SearchConfig::validate() const {
  for (double v : {max_tau, init_max_tau, max_vel, max_acc, w_time, horizon, lambda_heu,
                   resolution, time_resolution, safe_distance, voxel_size, collision_step, search_budget})
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("search parameters must be finite and positive");
  if (allocate_num < 2 || tree_period < 1 || max_cloud_points < 1 || !lower.allFinite() || !upper.allFinite() ||
      (lower.array() >= upper.array()).any() || ((upper-lower).array()/resolution > 1e8).any() ||
      collision_step > safe_distance || search_budget > 60 ||
      std::max(max_tau, init_max_tau)*std::sqrt(3.0)*max_vel/collision_step > 1e6)
    throw std::invalid_argument("invalid map bounds, capacities, sampling step or search budget");
}

KinodynamicAstar::KinodynamicAstar(const SearchConfig& c) : config_(c) {
  c.validate();
  max_tau_=c.max_tau; init_max_tau_=c.init_max_tau; max_vel_=c.max_vel; max_acc_=c.max_acc;
  w_time_=c.w_time; horizon_=c.horizon; lambda_heu_=c.lambda_heu; allocate_num_=c.allocate_num;
  resolution_=c.resolution; inv_resolution_=1/c.resolution;
  time_resolution_=c.time_resolution; inv_time_resolution_=1/c.time_resolution; origin_=c.lower;
  path_node_pool_.reserve(allocate_num_);
  for (int i=0; i<allocate_num_; ++i) path_node_pool_.push_back(new PathNode);
  clearMap();
}
KinodynamicAstar::~KinodynamicAstar() { for (auto node : path_node_pool_) delete node; }

void KinodynamicAstar::reset() {
  expanded_nodes_.clear(); path_nodes_.clear(); open_set_ = decltype(open_set_){};
  for (int i=0; i<use_node_num_; ++i) *path_node_pool_[i] = PathNode{};
  use_node_num_=0; iter_num_=0; is_shot_succ_=false; t_shot_=0;
}
void KinodynamicAstar::clearMap() {
  for (std::size_t i=0; i<2; ++i) {
    trees_[i] = pcl::search::KdTree<pcl::PointXYZ>();
    clouds_[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
  }
  cloud_input_num_=0; map_ready_=false;
  reset();
}
std::size_t KinodynamicAstar::mapPointCount() const { return clouds_[0]->size()+clouds_[1]->size(); }

// Preserve the two alternating, frame-accumulated trees; each bank lasts at most two periods.
bool KinodynamicAstar::setKdtree(const pcl::PointCloud<pcl::PointXYZ>& input) {
  if (input.empty() || input.size() > config_.max_cloud_points) { clearMap(); return false; }
  for (const auto& p : input)
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) { clearMap(); return false; }
  const auto bank = (cloud_input_num_/config_.tree_period)%2;
  auto next = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (cloud_input_num_%config_.tree_period != 0) *next = *clouds_[bank];
  *next += input;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(next);
  filter.setLeafSize(config_.voxel_size, config_.voxel_size, config_.voxel_size);
  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filter.filter(*filtered);
  if (filtered->empty() || filtered->size()+clouds_[1-bank]->size() > config_.max_cloud_points) {
    clearMap(); return false;
  }
  clouds_[bank] = filtered; trees_[bank].setInputCloud(filtered);
  ++cloud_input_num_; map_ready_=true;
  return true;
}
bool KinodynamicAstar::inBounds(const Eigen::Vector3d& p) const {
  return p.allFinite() && (p.array() >= config_.lower.array()).all() && (p.array() <= config_.upper.array()).all();
}
bool KinodynamicAstar::isSafe(double x, double y, double z) {
  if (!map_ready_ || !inBounds({x,y,z})) return false;
  // Centroid voxel displacement + half sampling interval conservatively cover between-sample collisions.
  const double radius = config_.safe_distance + std::sqrt(3.0)*config_.voxel_size + config_.collision_step/2;
  for (std::size_t i=0; i<2; ++i) {
    if (clouds_[i]->empty()) continue;
    if (trees_[i].nearestKSearch(pcl::PointXYZ(x,y,z), 1, nearest_indices_, nearest_distances_) != 1 ||
        nearest_distances_[0] <= radius*radius) return false;
  }
  return true;
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{

  
  reset();
  deadline_ = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config_.search_budget));
  if (!start_pt.allFinite() || !start_v.allFinite() || !start_a.allFinite() ||
      !end_pt.allFinite() || !end_v.allFinite() || !std::isfinite(time_start) ||
      !inBounds(start_pt) || !inBounds(end_pt) ||
      start_v.cwiseAbs().maxCoeff() > max_vel_ || end_v.cwiseAbs().maxCoeff() > max_vel_ ||
      start_a.cwiseAbs().maxCoeff() > max_acc_) return INVALID_INPUT;
  if (!map_ready_) return NO_MAP;
  if (!isSafe(start_pt.x(), start_pt.y(), start_pt.z()) ||
      !isSafe(end_pt.x(), end_pt.y(), end_pt.z())) return NO_PATH;
  time_origin_ = time_start;
  start_vel_ = start_v;
  start_acc_ = start_a;

  PathNodePtr cur_node = path_node_pool_[0];
  *cur_node = PathNode{};
  cur_node->time = time_start;
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push({cur_node->f_score, cur_node});
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);

  while (!open_set_.empty())
  {
    if (std::chrono::steady_clock::now() >= deadline_) { reset(); return TIMEOUT; }
    const auto entry = open_set_.top();
    cur_node = entry.node;
    if (cur_node->node_state != IN_OPEN_SET || entry.score != cur_node->f_score) {
      open_set_.pop();
      continue;
    }
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;

    if (reach_horizon || near_end)
    {
      terminate_node = cur_node;
      retrievePath(terminate_node);
      if (near_end)
      {
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        // The upstream heuristic time ignores derivative constraints. Retry the same cubic
        // boundary-value connection at longer durations, never accept an over-limit shot.
        for (double scale : {1.0, 1.5, 2.0, 3.0, 4.0}) {
          if (computeShotTraj(cur_node->state, end_state, time_to_goal * scale)) break;
        }
        if (std::chrono::steady_clock::now() >= deadline_) { reset(); return TIMEOUT; }
      }

    }
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        return REACH_END;
      }
      else
      {
        return REACH_HORIZON;
      }
    }

    if (near_end)
    {
      if (is_shot_succ_)
      {
        return REACH_END;
      }
    }
    path_nodes_.clear();
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }
    for (std::size_t i = 0; i < inputs.size(); ++i)
      for (std::size_t j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        if (std::chrono::steady_clock::now() >= deadline_) { reset(); return TIMEOUT; }
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        Eigen::Vector3d pro_pos = pro_state.head(3);
        if (!inBounds(pro_pos)) continue;
        bool leaves_bounds = false;
        for (int axis=0; axis<3; ++axis) {
          if (std::abs(um(axis)) < 1e-12) continue;
          const double t = -cur_state(axis+3)/um(axis);
          if (t>0 && t<tau) {
            const double p = cur_state(axis)+cur_state(axis+3)*t+0.5*um(axis)*t*t;
            leaves_bounds |= p<config_.lower(axis) || p>config_.upper(axis);
          }
        }
        if (leaves_bounds) continue;
        Eigen::Vector3i pro_id = posToIndex(pro_pos);

        int pro_t_id = dynamic ? timeToIndex(pro_t) : 0;
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
            continue;
        }
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
            continue;
        }
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
            continue;
        }
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        // Speed bounds give a spatial sampling bound even on returning/curved primitives.
        const int dyn_checknum = std::max(1, static_cast<int>(std::ceil(
            tau * std::sqrt(3.0) * max_vel_ / config_.collision_step)));
        for (int k = 1; k <= dyn_checknum; ++k)
        {
          double dt = tau * double(k) / double(dyn_checknum);

          stateTransit(cur_state, xt, um, dt);

          pos = xt.head(3);
          if (!inBounds(pos))
          {
            is_occ = true;
            break;
          }

          if (!isSafe(pos(0),pos(1),pos(2))) {
            is_occ = true;
            break;
          }

        }
        if (is_occ)
        {
            continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);
        bool prune = false;
        for (std::size_t j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              open_set_.push({expand_node->f_score, expand_node});
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }
        if (!prune)
        {
          if (pro_node == NULL)
          {
            if (use_node_num_ >= allocate_num_) { reset(); return NODE_LIMIT; }
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push({pro_node->f_score, pro_node});

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time_idx, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;

          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              open_set_.push({pro_node->f_score, pro_node});
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
        }
      }
  }
  return NO_PATH;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.clear();
  path_nodes_.push_back(cur_node);


  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);


  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = std::max(1e-3, (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max);
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (!std::isfinite(t) || t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  if (!std::isfinite(t_d) || t_d <= 0) return false;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  // Exact extrema: acceleration is linear, velocity quadratic, position cubic.
  for (int axis = 0; axis < 3; ++axis) {
    const auto position = [&](double t) { return d(axis) + c(axis)*t + b(axis)*t*t + a(axis)*t*t*t; };
    const auto velocity = [&](double t) { return c(axis) + 2*b(axis)*t + 3*a(axis)*t*t; };
    if (std::max(std::abs(2*b(axis)), std::abs(2*b(axis)+6*a(axis)*t_d)) > max_acc_ + 1e-9)
      return false;
    std::vector<double> velocity_times{0.0, t_d};
    if (std::abs(a(axis)) > 1e-12) velocity_times.push_back(-b(axis)/(3*a(axis)));
    for (double t : velocity_times)
      if (t >= 0 && t <= t_d && std::abs(velocity(t)) > max_vel_ + 1e-9) return false;
    std::vector<double> position_times{0.0, t_d};
    if (std::abs(a(axis)) > 1e-12) {
      const double disc = 4*b(axis)*b(axis)-12*a(axis)*c(axis);
      if (disc >= 0) {
        position_times.push_back((-2*b(axis)+std::sqrt(disc))/(6*a(axis)));
        position_times.push_back((-2*b(axis)-std::sqrt(disc))/(6*a(axis)));
      }
    } else if (std::abs(b(axis)) > 1e-12) position_times.push_back(-c(axis)/(2*b(axis)));
    for (double t : position_times)
      if (t >= 0 && t <= t_d && (position(t) < config_.lower(axis) || position(t) > config_.upper(axis)))
        return false;
  }
  const int checks = std::max(1, static_cast<int>(std::ceil(t_d * std::sqrt(3.0) * max_vel_ / config_.collision_step)));
  for (int i = 0; i <= checks; ++i) {
    if (std::chrono::steady_clock::now() >= deadline_) return false;
    const double t = t_d * i / checks;
    const Vector3d p = d + c*t + b*t*t + a*t*t*t;
    if (!isSafe(p.x(), p.y(), p.z())) return false;
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(std::clamp(R / sqrt(-Q * Q * Q), -1.0, 1.0));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  return static_cast<int>(std::floor((time - time_origin_) * inv_time_resolution_));
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

// Export the original constant-acceleration and terminal cubic segments, in execution order.
std::vector<TrajectorySegment> KinodynamicAstar::getSegments() const {
  std::vector<TrajectorySegment> out;
  for (const auto node : path_nodes_) {
    if (!node->parent) continue;
    TrajectorySegment s;
    s.duration = node->duration;
    s.coefficients.col(0) = node->parent->state.head<3>();
    s.coefficients.col(1) = node->parent->state.tail<3>();
    s.coefficients.col(2) = node->input/2;
    out.push_back(s);
  }
  if (is_shot_succ_) out.push_back({t_shot_, coef_shot_});
  return out;
}
std::vector<TrajectorySample> KinodynamicAstar::sampleTrajectory(double step) const {
  if (!std::isfinite(step) || step <= 0) throw std::invalid_argument("sample step must be positive");
  std::vector<TrajectorySample> out;
  double elapsed = 0;
  for (const auto& s : getSegments()) {
    const double count = std::ceil(s.duration/step);
    if (count > 1000000 || out.size()+count+1 > 1000000) throw std::length_error("too many samples");
    const int n = std::max(1, static_cast<int>(count));
    for (int i = out.empty() ? 0 : 1; i <= n; ++i) {
      const double t = s.duration*i/n;
      const auto& c = s.coefficients;
      out.push_back({elapsed+t, c.col(0)+c.col(1)*t+c.col(2)*t*t+c.col(3)*t*t*t,
          c.col(1)+2*c.col(2)*t+3*c.col(3)*t*t, 2*c.col(2)+6*c.col(3)*t});
    }
    elapsed += s.duration;
  }
  return out;
}
std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double step) const {
  std::vector<Eigen::Vector3d> out;
  for (const auto& sample : sampleTrajectory(step)) out.push_back(sample.position);
  return out;
}
std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes() {
  return {path_node_pool_.begin(), path_node_pool_.begin()+use_node_num_};
}

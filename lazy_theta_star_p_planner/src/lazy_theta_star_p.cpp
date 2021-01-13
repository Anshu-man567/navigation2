//  Copyright 2020 Anshumaan Singh
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  http:// www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "lazy_theta_star_p_planner/lazy_theta_star_p.hpp"
#include <vector>

namespace lazyThetaStarP {

LazyThetaStarP::LazyThetaStarP() {
  costmap_tolerance_ = 1.0;
  euc_tolerance_ = 1.0;
  how_many_corners_ = 8;
  size_x_ = 0;
  size_y_ = 0;
  index_generated = 0;
}

bool LazyThetaStarP::generatePath(std::vector<coordsW> & raw_path) {

  setContainers();

  RCLCPP_INFO(node_->get_logger(), "Path Planning Begins... ");

  index_generated = 0;
  int curr_id = index_generated;
  addToNodesData(index_generated);
  nodes_data[curr_id] = {src.x, src.y, getTraversalCost(src.x, src.y), dist(src.x, src.y, dst.x, dst.y),
						 curr_id, true, dist(src.x, src.y, dst.x, dst.y) + (getTraversalCost(src.x, src.y))};
  queue_.push({curr_id, (nodes_data[curr_id].f)});
  addIndex(nodes_data[curr_id].x, nodes_data[curr_id].y, index_generated);
  index_generated++;

  int nodes_opened = 0;

  auto start = std::chrono::steady_clock::now();
  while (!queue_.empty()) {

    nodes_opened++;

	if (isGoal(nodes_data[curr_id].x, nodes_data[curr_id].y)) {
	  break;
	}
	tree_node & curr_data = nodes_data[curr_id];
	resetParent(curr_data);
	setNeighbors(curr_data, curr_id);

	curr_id = queue_.top().pos_id;
	queue_.pop();

  }
  auto stop = std::chrono::steady_clock::now();
  auto dur_while = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout<< "duration for the algo is !! ----- " << dur_while.count() << '\n';

  if (queue_.empty()) {
	RCLCPP_INFO(node_->get_logger(), "No Path Found !!!!!!!!!!!");
	raw_path.clear();
	return false;
  }



  std::cout << "the number of executions are: " << nodes_opened << '\n';
  RCLCPP_INFO(node_->get_logger(),
			  "REACHED DEST  %i,   %i --- %f \n the number of nodes are %i" ,
			  nodes_data[curr_id].x,
			  nodes_data[curr_id].y,
			  nodes_data[curr_id].g,
			  nodes_data.size());
  backtrace(raw_path, curr_id);
  clearQueue();


  return true;
}

void LazyThetaStarP::resetParent(tree_node & curr_data) {
  double g_cost, los_cost = 0;
  curr_data.is_in_queue = false;
  tree_node & curr_par = nodes_data[curr_data.parent_id];
  tree_node & maybe_par = nodes_data[curr_par.parent_id];

  if (losCheck(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y, los_cost)) {
	g_cost = maybe_par.g +
		getEucledianCost(curr_data.x, curr_data.y, maybe_par.x, maybe_par.y) +
		los_cost;
//	cal_cost = g_cost + curr_data.h;
	if (g_cost < curr_data.g) {
	  curr_data.parent_id = curr_par.parent_id;
	  curr_data.g = g_cost;
	  curr_data.f = g_cost + curr_data.h;
	}
  }
}

void LazyThetaStarP::setNeighbors(const tree_node & curr_data, const int & curr_id) {
  int mx, my;
  int m_id, m_par;
  double g_cost, h_cost, cal_cost;

//  const tree_node & curr_par = nodes_data[curr_data.parent_id];
  for (int i = 0; i < how_many_corners_; i++) {
	mx = curr_data.x + moves[i].x;
	my = curr_data.y + moves[i].y;

//	if (mx == curr_par.x && my == curr_par.y) {
//	  continue;
//	}

	if (withinLimits(mx, my)) {
	  if (!isSafe(mx, my)) {
		continue;
	  }
	} else {
	  continue;
	}

	m_par = curr_id;
	// TODO (Anshu-man567) : streamline the cost function in here by making a look table according to the
	// 						iter number, make it dependant on "i"
	g_cost = curr_data.g + getEucledianCost(curr_data.x, curr_data.y, mx, my) +
		getTraversalCost(mx, my);

	getIndex(mx, my, m_id);

	if (m_id == -1) {
	  addToNodesData(index_generated);
	  m_id = index_generated;
	  addIndex(mx, my, m_id);
	  index_generated++;
	}

	curr_node = &nodes_data[m_id];

	h_cost = getEucledianCost(mx, my, dst.x, dst.y);
	cal_cost = g_cost + h_cost;

	if (curr_node->f > cal_cost) {
	  curr_node->g = g_cost;
	  curr_node->h = h_cost;
	  curr_node->f = cal_cost;
	  curr_node->parent_id = m_par;
	  if (!curr_node->is_in_queue) {
		curr_node->x = mx;
		curr_node->y = my;
		curr_node->is_in_queue = true;
		queue_.push({m_id, (curr_node->f)});
	  }
	}
  }
}

void LazyThetaStarP::backtrace(std::vector<coordsW> & raw_points, int curr_id) {
  std::vector<coordsW> path_rev;
  /// added to get the path length
  std::vector<coordsM> path;
  coordsW world{};
  double pusher_ = 0.5 * costmap_->getResolution();
  do {
	path.push_back({nodes_data[curr_id].x, nodes_data[curr_id].y});
	costmap_->mapToWorld(nodes_data[curr_id].x, nodes_data[curr_id].y, world.x, world.y);
	path_rev.push_back({world.x + pusher_, world.y + pusher_});
	if (path_rev.size() > 1)
//	  std::cout << "the losCheck between " << nodes_data[curr_id].x << '\t' << nodes_data[curr_id].y << "\t & "
//				<< nodes_data[nodes_data[curr_id].parent_id].x << '\t' << nodes_data[nodes_data[curr_id].parent_id].y
//				<< "   " <<
//				losCheck(nodes_data[curr_id].x,
//						 nodes_data[curr_id].y,
//						 nodes_data[nodes_data[curr_id].parent_id].x,
//						 nodes_data[nodes_data[curr_id].parent_id].y,
//						 cost) << '\n';
	curr_id = nodes_data[curr_id].parent_id;
  } while (curr_id != 0);
  path.push_back({nodes_data[curr_id].x, nodes_data[curr_id].y});
  costmap_->mapToWorld(nodes_data[curr_id].x, nodes_data[curr_id].y, world.x, world.y);
  path_rev.push_back({world.x + pusher_, world.y + pusher_});

  double tot_euc_cost = 0.0;
  for(int i = 0; i < int(path.size()) - 1; i++) {
//	std::cout<< "between " << path[i].x << '\t' << path[i].y << '\t' << path[i+1].x << '\t' << path[i+1].y<< '\t' <<
//			dist(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y) << '\n';
    tot_euc_cost += dist(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
  }

  RCLCPP_INFO(
	  node_->get_logger(), "Got the src and dst... (%i, %i) & (%i, %i) \n PATH LENGTH is ============================ %f",
	 src.x, src.y, dst.x, dst.y, tot_euc_cost);

  raw_points.reserve(path_rev.size());
  for (int i = static_cast<int>(path_rev.size()) - 1; i >= 0; i--) {
    raw_points.push_back(path_rev[i]);
  }
}

bool LazyThetaStarP::losCheck(
	const int & x0,
	const int & y0,
	const int & x1,
	const int & y1,
	double & sl_cost) {
  sl_cost = 0;

//  if(!( isSafe(x0,y0) && isSafe(x1,y1)))
//    return false;

//  std::cout<<"for losCheck between " << x0 << '\t' << y0 << '\t' << x1 << '\t' << y1 << '\n';

  int cx, cy;
  int dy = y1 - y0, dx = x1 - x0, f = 0;
  int sx, sy;

  int lx, ly;

  if (dy < 0) {
	dy = -dy;
	sy = -1;
  } else {
	sy = 1;
  }

  if (dx < 0) {
	dx = -dx;
	sx = -1;
  } else {
	sx = 1;
  }

  int u_x = (sx - 1) / 2;
  int u_y = (sy - 1) / 2;
  lx = x1; // + u_x;
  ly = y1; // + u_y;
  cx = x0;
  cy = y0;

  if (dx >= dy) {
	while (cx != lx) {
	  f += dy;
	  if (f >= dx) {
		if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
		  return false;
		}
		cy += sy;
		f -= dx;
	  }
	  if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
		return false;
	  }
	  if (dy == 0 && !isSafe(cx + u_x, cy, sl_cost) && !isSafe(cx + u_x, cy - 1, sl_cost)) {
		return false;
	  }
	  cx += sx;
	}
  } else {
	while (cy != ly) {
	  f = f + dx;
	  if (f >= dy) {
		if (!isSafe(cx + u_x, cy + u_y, sl_cost)) {
		  return false;
		}
		cx += sx;
		f -= dy;
	  }
	  if (f != 0 && !isSafe(cx + u_x, cy + u_y, sl_cost)) {
		  return false;
		}
	  if (dx == 0 && isSafe(cx, cy + u_y, sl_cost) && !isSafe(cx - 1, cy + u_y, sl_cost)) {
		return false;
	  }
	  cy += sy;
	}
  }
  return true;
}

void LazyThetaStarP::setContainers() {
  auto start = std::chrono::steady_clock::now();
  index_generated = 0;
  int last_size_x = size_x_;
  int last_size_y = size_y_;
  int curr_size_x = static_cast<int>(costmap_->getSizeInCellsX());
  int curr_size_y = static_cast<int>(costmap_->getSizeInCellsY());
  if (last_size_x != curr_size_x || last_size_y != curr_size_y) {
	initializePosn(curr_size_y * curr_size_x - last_size_y * last_size_x);
	nodes_data.reserve(curr_size_x * curr_size_y);
  } else {
	initializePosn();
  }
  size_x_ = curr_size_x;
  size_y_ = curr_size_y;
  auto stop = std::chrono::steady_clock::now();
  auto dur_init = (std::chrono::duration_cast<std::chrono::microseconds>(stop - start));
  std::cout<< "dur_init --- " << dur_init.count() << '\n';
}
}  //  namespace lazyThetaStarP

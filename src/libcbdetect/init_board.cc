/**
* Copyright 2018, ftdlyc <yclu.cn@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

/*
% Copyright 2012. All rights reserved.
% Author: Andreas Geiger
%         Institute of Measurement and Control Systems (MRT)
%         Karlsruhe Institute of Technology (KIT), Germany

% This is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% This software is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#include <vector>

#include "libcbdetect/config.h"

namespace cbdetect {

int directional_neighbor(const Corner& corners, const std::vector<int>& used,
                         int idx, const cv::Point2d& v, double& min_dist) {
  //创建角点数量大小的vector,并用1e10初始化
  std::vector<double> dists(corners.p.size(), 1e10);

  // distances
  for(int i = 0; i < corners.p.size(); ++i) {
    if(used[i]) {
      continue;
    }
    cv::Point2d dir   = corners.p[i] - corners.p[idx];
    double dist_point = dir.x * v.x + dir.y * v.y;
    dir               = dir - dist_point * v;
    double dist_edge  = cv::norm(dir);
    double dist       = dist_point + 5 * dist_edge;
    if(dist_point >= 0) {
      dists[i] = dist;
    }
  }

  // find best neighbor
  int neighbor_idx = std::min_element(dists.begin(), dists.end()) - dists.begin();
  min_dist         = dists[neighbor_idx];
  return neighbor_idx;
}

bool init_board(const Corner& corners, std::vector<int>& used, Board& board, int idx) {
  board.idx.clear();
  // return if not enough corners
  //如果角点的数量小于9个就返回fasle
  if(corners.p.size() < 9) {
    return false;
  }

  // init chessboard hypothesis
  //初始化棋盘格
  board.idx = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  // extract feature index and orientation (central element)
  //提取中心元素的索引和方向
  //v1向右为正，v2向下为正
  const cv::Point2d& v1 = corners.v1[idx];
  const cv::Point2d& v2 = corners.v3.empty() ? corners.v2[idx] : corners.v3[idx];
  board.idx[1][1]       = idx;
  used[idx]             = 1;
  double min_dist[8];

  // find left/right/top/bottom neighbors
  //查找左、右、上、下四个方向上的最近邻
  board.idx[1][0]       = directional_neighbor(corners, used, idx, -v1, min_dist[0]);
  used[board.idx[1][0]] = 1;
  board.idx[1][2]       = directional_neighbor(corners, used, idx, v1, min_dist[1]);
  used[board.idx[1][2]] = 1;
  board.idx[0][1]       = directional_neighbor(corners, used, idx, -v2, min_dist[2]);
  used[board.idx[0][1]] = 1;
  board.idx[2][1]       = directional_neighbor(corners, used, idx, v2, min_dist[3]);
  used[board.idx[2][1]] = 1;

  // find top-left/top-right/bottom-left/bottom-right neighbors
  //查找左上、右上、左下、右下的最近邻
  //左上：左侧近邻的上方最近点    上侧近邻的左方最近点
  //如果两个方向查找到的点的索引相同，直接取tmp1
  //如果两个方向查找到的点的索引不相同，比较两个点两个点距离左侧点和上侧点的距离差，谁小就将谁作为tmp1
  int tmp1, tmp2;
  double d1, d2, min_dist_tmp1, min_dist_tmp2;
  tmp1 = directional_neighbor(corners, used, board.idx[1][0], -v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[0][1], -v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[board.idx[1][0]]) -
                  cv::norm(corners.p[tmp1] - corners.p[board.idx[0][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[board.idx[1][0]]) -
                  cv::norm(corners.p[tmp2] - corners.p[board.idx[0][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[0][0] = tmp1;
  min_dist[4]     = min_dist_tmp1;
  used[tmp1]      = 1;

  //右上最近邻的查找如法炮制
  tmp1 = directional_neighbor(corners, used, board.idx[1][2], -v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[0][1], v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[board.idx[1][2]]) -
                  cv::norm(corners.p[tmp1] - corners.p[board.idx[0][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[board.idx[1][2]]) -
                  cv::norm(corners.p[tmp2] - corners.p[board.idx[0][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[0][2] = tmp1;
  min_dist[5]     = min_dist_tmp1;
  used[tmp1]      = 1;
  
  //左下最近邻的查找如法炮制
  tmp1 = directional_neighbor(corners, used, board.idx[1][0], v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[2][1], -v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[board.idx[1][0]]) -
                  cv::norm(corners.p[tmp1] - corners.p[board.idx[2][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[board.idx[1][0]]) -
                  cv::norm(corners.p[tmp2] - corners.p[board.idx[2][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[2][0] = tmp1;
  min_dist[6]     = min_dist_tmp1;
  used[tmp1]      = 1;

  //右下最近邻的查找如法炮制
  tmp1 = directional_neighbor(corners, used, board.idx[1][2], v2, min_dist_tmp1);
  tmp2 = directional_neighbor(corners, used, board.idx[2][1], v1, min_dist_tmp2);
  if(tmp1 != tmp2) {
    d1 = std::abs(cv::norm(corners.p[tmp1] - corners.p[board.idx[1][2]]) -
                  cv::norm(corners.p[tmp1] - corners.p[board.idx[2][1]]));
    d2 = std::abs(cv::norm(corners.p[tmp2] - corners.p[board.idx[1][2]]) -
                  cv::norm(corners.p[tmp2] - corners.p[board.idx[2][1]]));
    if(d1 > d2) {
      std::swap(tmp1, tmp2);
      std::swap(min_dist_tmp1, min_dist_tmp2);
    }
  }
  board.idx[2][2] = tmp1;
  min_dist[7]     = min_dist_tmp1;
  used[tmp1]      = 1;

  // initialization must be homogenously distributed
  //初始化必须均匀分布
  //遍历idx周围的8个最近邻的min_dist，如果有任何一个最近邻的min_dist与1e10非常接近，即没有找到最近邻，就将该idx清除，返回false
  //这样做避免了随机获取的idx找到了图像边缘一圈
  for(int i = 0; i < 8; ++i) {
    if(std::abs(min_dist[i] - 1e10) < 1) {
      for(int jj = 0; jj < 3; ++jj) {
        for(int ii = 0; ii < 3; ++ii) {
          used[board.idx[jj][ii]] = 0;
        }
      }
      board.idx.clear();
      return false;
    }
  }

  board.num    = 9;
  board.energy = std::move(
      std::vector<std::vector<std::vector<double>>>(3,
                                                    std::vector<std::vector<double>>(3,
                                                                                     std::vector<double>(3, DBL_MAX))));
  return true;
}

} // namespace cbdetect
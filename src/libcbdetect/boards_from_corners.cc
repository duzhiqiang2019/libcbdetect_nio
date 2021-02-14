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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "libcbdetect/board_energy.h"
#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/filter_board.h"
#include "libcbdetect/grow_board.h"
#include "libcbdetect/init_board.h"

namespace cbdetect {
/*
int directional_neighbor_df(const Corner& corners, const std::vector<int>& used,
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
  min_dist   = dists[neighbor_idx];
  return neighbor_idx;
}
*/

void debug_grow_process(const cv::Mat& img, const Corner& corners, const Board& board,
                        const std::vector<cv::Point2i>& proposal, int direction, bool type) {
  cv::Mat img_show;
  if(img.channels() != 3) {
#if CV_VERSION_MAJOR >= 4
    cv::cvtColor(img, img_show, cv::COLOR_GRAY2BGR);
#else
    cv::cvtColor(img, img_show, CV_GRAY2BGR);
#endif
  } else {
    img_show = img.clone();
  }

  cv::Point2d mean(0.0, 0.0);
  for(int i = 0; i < board.idx.size(); ++i) {
    for(int j = 0; j < board.idx[i].size(); ++j) {
      if(board.idx[i][j] < 0) {
        continue;
      }
      cv::circle(img_show, corners.p[board.idx[i][j]], 4, cv::Scalar(255, 0, 0), -1);
      cv::putText(img_show, std::to_string(board.idx[i][j]),
                  cv::Point2i(corners.p[board.idx[i][j]].x - 12, corners.p[board.idx[i][j]].y - 6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
      mean += corners.p[board.idx[i][j]];
    }
  }
  mean /= (double)(board.num);
  mean.x -= 10;
  mean.y += 10;
  cv::putText(img_show, std::to_string(direction), mean,
              cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(196, 196, 0), 2);

  for(const auto& i : proposal) {
    if(board.idx[i.y][i.x] < 0) {
      continue;
    }
    if(type) {
      cv::circle(img_show, corners.p[board.idx[i.y][i.x]], 4, cv::Scalar(0, 255, 0), -1);
    } else {
      cv::circle(img_show, corners.p[board.idx[i.y][i.x]], 4, cv::Scalar(0, 0, 255), -1);
    }
  }

  cv::imshow("grow_process", img_show);
  cv::waitKey();
}

void boards_from_corners(const cv::Mat& img, const Corner& corners, std::vector<Board>& boards, const Params& params) {
  // intialize boards
  //先将boards清空
  boards.clear();
  //新创建一个board对象
  Board board;
  //新建一个used数组，数组的大小与corners中角点的个数相同，初始化为0
  std::vector<int> used(corners.p.size(), 0);

  //新建一个int类型的start,初始化为0
  int start = 0;
  if(!params.overlay) {
    // start from random index
    //创建随机数
    std::default_random_engine e;
    //取时间
    auto time = std::chrono::system_clock::now().time_since_epoch();
    //将时间作为随机数种子
    e.seed(static_cast<unsigned long>(time.count()));
    //使用随机数对start进行初始化
    start = e() % corners.p.size();
  }

  // for all seed corners do
  //遍历所有的corners
  int n = 0;
  while(n++ < corners.p.size()) {
    // init 3x3 board from seed i
    int i = (n + start) % corners.p.size();
    if(used[i] == 1 || !init_board(corners, used, board, i)) {
      continue;
    }

    // check if this is a useful initial guess
    cv::Point3i maxE_pos = board_energy(corners, board, params);
    double energy        = board.energy[maxE_pos.y][maxE_pos.x][maxE_pos.z];
    if(energy > -6.0) {
      for(int jj = 0; jj < 3; ++jj) {
        for(int ii = 0; ii < 3; ++ii) {
          used[board.idx[jj][ii]] = 0;
        }
      }
      continue;
    }

    // grow boards
    while(1) {
      int num_corners = board.num;

      for(int j = 0; j < (params.corner_type == MonkeySaddlePoint ? 6 : 4); ++j) {
        std::vector<cv::Point2i> proposal;
        GrowType grow_type = grow_board(corners, used, board, proposal, j, params);
        if(grow_type == GrowType_Failure) {
          continue;
        }

        if(params.show_grow_processing) {
          for(int ii = 0; ii < board.idx.size(); ++ii) {
            for(int jj = 0; jj < board.idx[ii].size(); ++jj) {
              std::cout << board.idx[ii][jj] << " ";
            }
            std::cout << "\n";
          }
          std::cout << "\n";
          debug_grow_process(img, corners, board, proposal, j, false);
        }

        filter_board(corners, used, board, proposal, energy, params);

        if(params.show_grow_processing) {
          for(int ii = 0; ii < board.idx.size(); ++ii) {
            for(int jj = 0; jj < board.idx[ii].size(); ++jj) {
              std::cout << board.idx[ii][jj] << " ";
            }
            std::cout << "\n";
          }
          std::cout << "\n";
          debug_grow_process(img, corners, board, proposal, j, true);
        }

        if(grow_type == GrowType_Inside) {
          --j;
        }
      }

      // exit loop
      if(board.num == num_corners) {
        break;
      }
    }

    if(!params.overlay) {
      boards.emplace_back(board);
      continue;
    }

    std::vector<std::pair<int, double>> overlap;
    for(int j = 0; j < boards.size(); ++j) {
      // check if new chessboard proposal overlaps with existing chessboards
      for(int k1 = 0; k1 < board.idx.size(); ++k1) {
        for(int k2 = 0; k2 < board.idx[0].size(); ++k2) {
          for(int l1 = 0; l1 < boards[j].idx.size(); ++l1) {
            for(int l2 = 0; l2 < boards[j].idx[0].size(); ++l2) {
              if(board.idx[k1][k2] != -1 && board.idx[k1][k2] != -2 && board.idx[k1][k2] == boards[j].idx[l1][l2]) {
                cv::Point3i maxE_pos_tmp = board_energy(corners, boards[j], params);
                overlap.emplace_back(std::make_pair(j, boards[j].energy[maxE_pos_tmp.y][maxE_pos_tmp.x][maxE_pos_tmp.z]));
                goto GOTO_BREAK;
              }
            }
          }
        }
      }
    }
  GOTO_BREAK:;

    if(overlap.empty()) {
      boards.emplace_back(board);
    } else {
      bool is_better = true;
      for(int j = 0; j < overlap.size(); ++j) {
        if(overlap[j].second <= energy) {
          is_better = false;
          break;
        }
      }
      if(is_better) {
        std::vector<Board> tmp;
        for(int j = 0, k = 0; j < boards.size(); ++j) {
          if(overlap[k].first == j) {
            continue;
            ++k;
          }
          tmp.emplace_back(boards[j]);
        }
        std::swap(tmp, boards);
        boards.emplace_back(board);
      }
    }
    std::fill(used.begin(), used.end(), 0);
    n += 2;
  }
}

void boards_from_corners_df(const cv::Mat& img, const Corner& corners, std::vector<Board>& boards, const Params& params) 
{
  //新建corner用来存放非boards中的角点
  Corner cornersNew;
  Corner cornersNewLeftDown;
  Corner cornersNewLeftUp;
  Corner cornersNewRightDown;
  Corner cornersNewRightUp;
  //临时存放每个board
  Board tempBoardleft;
  Board tempBoardright;
  tempBoardleft.idx = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  tempBoardright.idx = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
  //创建哈希表用于筛选非boards中的角点
  std::unordered_map<double,double> table;
  //先将boards中的角点push进哈希表中
  int raw = boards[0].idx.size();
  int col = boards[0].idx[0].size();
  for(int i = 1; i < raw - 1; i++)
  {
    for(int j = 1; j < col - 1; j++)
    {
      table[corners.p[boards[0].idx[i][j]].x] = corners.p[boards[0].idx[i][j]].y;
    }
  }
  //遍历所有角点，如果哈希表中找不到该角点就将该角点放入cornersNew中
  int num = 0;
  for(int n = 0; n < corners.p.size(); n++)
  {
    if(table.find(corners.p[n].x) == table.end() || table[corners.p[n].x] != corners.p[n].y)
    {
      if(corners.p[n].y > 260 && corners.p[n].y < 550)
      {
        cornersNew.p.emplace_back(corners.p[n]);
        //使用cornersNew中的v1来存储元素在原corners中的索引
        cornersNew.r.emplace_back(n);
      }
    }
  }
  if(cornersNew.p.empty()) return;
  //std::cout<<cornersNew.p.size()<<std::endl;
  //遍历获得x坐标均值
  double xMean = 0.0;
  double yMean = 0.0;
  for(int i = 0; i < cornersNew.p.size(); i++)
  {
      xMean += cornersNew.p[i].x;
      yMean += cornersNew.p[i].y;
  }
  xMean /= cornersNew.p.size();
  yMean /= cornersNew.p.size();
  //根据x坐标的大小将角点分为左右两类
  for(int i = 0; i < cornersNew.p.size(); i++)
  {
    if(cornersNew.p[i].x < xMean)
    {
      //用v1.x来表示上下
      if(cornersNew.p[i].y < yMean )
      {
        cornersNewLeftUp.p.emplace_back(cornersNew.p[i]);
        cornersNewLeftUp.r.emplace_back(cornersNew.r[i]);
      }
      if(cornersNew.p[i].y > yMean )
      {
        cornersNewLeftDown.p.emplace_back(cornersNew.p[i]);
        cornersNewLeftDown.r.emplace_back(cornersNew.r[i]);
      }
    }
    else
    {
       if(cornersNew.p[i].y < yMean )
      {
        cornersNewRightUp.p.emplace_back(cornersNew.p[i]);
        cornersNewRightUp.r.emplace_back(cornersNew.r[i]);
      }
      if(cornersNew.p[i].y > yMean )
      {
        cornersNewRightDown.p.emplace_back(cornersNew.p[i]);
        cornersNewRightDown.r.emplace_back(cornersNew.r[i]);
      }
    }
  }
  //std::cout<<cornersNewLeftUp.p.size()<<" "<<cornersNewLeftDown.p.size()<<" "<<cornersNewRightUp.p.size()<<" "<<cornersNewRightDown.p.size()<<std::endl;

  tempBoardleft.idx[1][1] = cornersNewLeftDown.p[0].x < cornersNewLeftDown.p[1].x ? cornersNewLeftDown.r[0] : cornersNewLeftDown.r[1];
  tempBoardleft.idx[2][1] = cornersNewLeftDown.p[0].x > cornersNewLeftDown.p[1].x ? cornersNewLeftDown.r[0] : cornersNewLeftDown.r[1];
  tempBoardleft.idx[1][2] = cornersNewLeftUp.p[0].x < cornersNewLeftUp.p[1].x ? cornersNewLeftUp.r[0] : cornersNewLeftUp.r[1];
  tempBoardleft.idx[2][2] = cornersNewLeftUp.p[0].x > cornersNewLeftUp.p[1].x ? cornersNewLeftUp.r[0] : cornersNewLeftUp.r[1];
  tempBoardleft.num = 4;

  tempBoardright.idx[1][1] = cornersNewRightDown.p[0].x < cornersNewRightDown.p[1].x ? cornersNewRightDown.r[0] : cornersNewRightDown.r[1];
  tempBoardright.idx[2][1] = cornersNewRightDown.p[0].x > cornersNewRightDown.p[1].x ? cornersNewRightDown.r[0] : cornersNewRightDown.r[1];
  tempBoardright.idx[1][2] = cornersNewRightUp.p[0].x < cornersNewRightUp.p[1].x ? cornersNewRightUp.r[0] : cornersNewRightUp.r[1];
  tempBoardright.idx[2][2] = cornersNewRightUp.p[0].x > cornersNewRightUp.p[1].x ? cornersNewRightUp.r[0] : cornersNewRightUp.r[1];
  tempBoardright.num = 4;

  boards.emplace_back(tempBoardleft);
  boards.emplace_back(tempBoardright);

}
} // namespace cbdetect

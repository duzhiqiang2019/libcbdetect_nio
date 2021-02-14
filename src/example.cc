#include "../include/libcbdetect/boards_from_corners.h"
#include "../include/libcbdetect/config.h"
#include "../include/libcbdetect/find_corners.h"
#include "../include/libcbdetect/plot_boards.h"
#include "../include/libcbdetect/plot_corners.h"
#include"../include/libcbdetect/init_board.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>
#include<string>

using namespace std::chrono;
using namespace cbdetect;

void printBoards(std::string str,std::vector<cbdetect::Board>& boards,cbdetect::Corner& corners)
{
  cv::Mat img = cv::imread(str, cv::IMREAD_COLOR);
  int indexN = 0;
  int numBoard = boards.size();
  
  for(int n = 0; n < numBoard; ++n)
  {
    const auto& board = boards[n];
    std::cout<<"Boards:"<<n<<std::endl;
    for(int i = 1; i < board.idx[0].size() - 1; i++)
    {
      std::cout<<"[";
      for(int j = 1; j < board.idx.size() - 1; j++)
      {
        std::cout<<"("<<corners.p[board.idx[j][i]].x<<","<<corners.p[board.idx[j][i]].y<<")";
        cv::putText(img,std::to_string(indexN),corners.p[board.idx[j][i]],cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(10, 10, 0), 2);
        indexN++;
      }
      std::cout<<"]"<<std::endl;
    }
  }
  cv::imshow("DZQ", img);
  cv::waitKey();
}

void detect(std::string str, cbdetect::CornerType corner_type, cbdetect::Corner& corners,std::vector<cbdetect::Board>& boards,cbdetect::Params& params) {

  params.corner_type = corner_type;

  cv::Mat img = cv::imread(str, cv::IMREAD_COLOR);

  auto t1 = high_resolution_clock::now();
  //查找图片中所有的角点
  cbdetect::find_corners(img, corners, params);
  auto t2 = high_resolution_clock::now();
  //绘制图片中的角点
  cbdetect::plot_corners(img, corners);
  auto t3 = high_resolution_clock::now();
  //由角点生成棋盘格，这里只能生成角点数>=9的棋盘格
  std::cout<<corners.p[20].x<<" "<<corners.p[20].y<<std::endl;
  std::cout<<corners.p[21].x<<" "<<corners.p[21].y<<std::endl;
  cbdetect::boards_from_corners(img, corners, boards, params);
  auto t4 = high_resolution_clock::now();
  printf("Find corners took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0);
  printf("Find boards took: %.3f ms\n", duration_cast<microseconds>(t4 - t3).count() / 1000.0);
  printf("Total took: %.3f ms\n", duration_cast<microseconds>(t2 - t1).count() / 1000.0 + duration_cast<microseconds>(t4 - t3).count() / 1000.0);
  cbdetect::boards_from_corners_df(img,corners,boards,params);
  cbdetect::plot_boards(img, corners, boards, params);
  


}

int main(int argc, char* argv[]) {
  /*
  目前该棋盘格检测代码只使用于检测360全景标定图和单张棋盘格图片
  */
  cbdetect::Corner corners;
  std::vector<cbdetect::Board> boards;
  std::vector<cbdetect::Board> boardsNew;
  cbdetect::Params params;
  std::cout<<"测试360标定图像"<<std::endl;
  std::string str = "/home/nio/libcbdetect/example_data/1.png";
  detect(str,cbdetect::SaddlePoint,corners,boards,params);
  printBoards(str,boards,corners);
  // detect("/home/nio/libcbdetect/example_data/2.png",cbdetect::SaddlePoint,corners,boards,params);
  // printBoards(boards,corners);
  // detect("/home/nio/libcbdetect/example_data/3.png",cbdetect::SaddlePoint,corners,boards,params);
  // printBoards(boards,corners);
  // detect("/home/nio/libcbdetect/example_data/4.png",cbdetect::SaddlePoint,corners,boards,params);
  // printBoards(boards,corners);
  // detect("/home/nio/libcbdetect/example_data/5.png",cbdetect::SaddlePoint,corners,boards,params);
  // printBoards(boards,corners);

  // std::cout<<"测试单张棋盘格图纸"<<std::endl;
  // detect("/home/nio/libcbdetect/example_data/image.bmp",cbdetect::SaddlePoint,corners,boards,params);
  // printBoards(boards,corners);
  // detect("/home/nio/libcbdetect/example_data/1.bmp",cbdetect::SaddlePoint,corners,boards,params);
  // printBoards(boards,corners);
  // detect("/home/nio/libcbdetect/example_data/2.bmp",cbdetect::SaddlePoint,corners,boards,params);
  // printBoards(boards,corners);
  return 0;
}

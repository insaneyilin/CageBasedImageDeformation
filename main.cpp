// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: main.cc

#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cage_deform_tool/cage_deform_tool.h"

void PrintUsage() {
  std::cout << "Usage: ./cage_based_image_deformation "
      << "<input_image> <orig_cage_points_file> "
      << "<transformed_cage_points_file>\n"; 
}

void LoadCagePoints(const std::string &filename,
    std::vector<cv::Point2f> *points) {
  points->clear();
  std::ifstream ifs(filename);
  float x;
  float y;
  while (ifs >> x >> y) {
    points->push_back(cv::Point2f(x, y));
  }
}

void DrawCagePoints(const std::vector<cv::Point2f> &points,
    cv::Mat *img) {
  const int num_pts = points.size();
  for (int i = 0; i < num_pts; ++i) {
    const auto &pt = points[i];
    const auto &next_pt = points[(i + 1) % num_pts];
    cv::circle(*img, pt, 3, cv::Scalar(0, 0, 255));
    cv::line(*img, pt, next_pt, cv::Scalar(0, 255, 0));
  }
}

int main(int argc, char **argv) {
  if (argc != 4) {
    PrintUsage();
    return 1;
  }
  std::string input_img_filepath(argv[1]);
  std::string orig_cage_points_file(argv[2]);
  std::string transformed_cage_points_file(argv[3]);
  std::vector<cv::Point2f> orig_points;
  std::vector<cv::Point2f> trans_points;
  LoadCagePoints(orig_cage_points_file, &orig_points);
  LoadCagePoints(transformed_cage_points_file, &trans_points);

  cv::Mat orig_image = cv::imread(input_img_filepath);
  cv::Mat img_orig_cage = orig_image.clone();
  cv::Mat img_trans_cage = orig_image.clone();

  // deformation
  cage_deform_tool::CageDeformTool cage_tool;
  cage_tool.SetAnchorPoints(orig_points, trans_points);
  cage_tool.Deform(img_orig_cage, &img_trans_cage);

  // visualization
  DrawCagePoints(orig_points, &img_orig_cage);
  DrawCagePoints(trans_points, &img_trans_cage);

  cv::imshow("Original Cage", img_orig_cage);
  cv::imshow("Transformed Cage", img_trans_cage);
  cv::waitKey(0);

  return 0;
}

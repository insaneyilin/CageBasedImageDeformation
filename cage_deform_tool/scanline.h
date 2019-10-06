// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: scanline.h

#ifndef SCANLINE_H_
#define SCANLINE_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cage_deform_tool {

class ScanLineAlgorithm {
 public:
  static void GetInternalPointsOfPolygon(
      const std::vector<cv::Point2f> &polygon,
      std::vector<cv::Point2i> *points);

  struct Edge {
    float x;
    float k;
    int ymax;
  };

 private:
  ScanLineAlgorithm() = default;
  ~ScanLineAlgorithm() = default;

  static void AddEdge(int x1, int y1, int x2, int y2);

 private:
  static int s_y_min_;
  static int s_y_max_;

  // edge table
  static std::vector<std::vector<Edge> > s_et_;

  // active edge table
  static std::vector<Edge> s_aet_;
};

}  // namespace cage_deform_tool

#endif  // SCANLINE_H_

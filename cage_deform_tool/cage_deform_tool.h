// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: cage_deform_tool.h

#ifndef CAGE_DEFORM_TOOL_H_
#define CAGE_DEFORM_TOOL_H_

#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cage_deform_tool {

class CageDeformTool {
 public:
  CageDeformTool() = default;
  ~CageDeformTool() = default;

  void SetAnchorPoints(const std::vector<cv::Point2f> &source_pts,
      const std::vector<cv::Point2f> &target_pts);

  void Deform(const cv::Mat &src, cv::Mat *dst);

 private:
  int GetSourcePointIndex(const cv::Point2f &pt);

  void CalcMeanValueBaryCentricCoordinates(const cv::Point2f &pt,
      std::vector<float> *weights);
  cv::Point2i GetPointByBaryCentricCoordinates(const cv::Point2f &pt,
      const std::vector<float> &weights);

 private:
  std::vector<cv::Point2f> source_points_;
  std::vector<cv::Point2f> target_points_;

  std::unique_ptr<cv::Subdiv2D> triangulation_ = nullptr;
};

}  // namespace cage_deform_tool

#endif  // CAGE_DEFORM_TOOL_H_

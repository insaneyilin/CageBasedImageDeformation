// Copyright (c) 2019 Yilin Gui. All rights reserved.
//
// filename: cage_deform_tool.cc

#include "cage_deform_tool/cage_deform_tool.h"

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "cage_deform_tool/scanline.h"

namespace cage_deform_tool {

static float angleBetween(const cv::Point2f &v1,
    const cv::Point2f &v2) {
  float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
  float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

  float dot = v1.x * v2.x + v1.y * v2.y;

  float a = dot / (len1 * len2);

  if (a >= 1.f) {
    return 0.f;
  } else if (a <= -1.f) {
    return M_PI;
  } else {
    return std::acos(a);
  }
}

void CageDeformTool::SetAnchorPoints(
    const std::vector<cv::Point2f> &source_pts,
    const std::vector<cv::Point2f> &target_pts) {
  source_points_.clear();
  target_points_.clear();
  if (source_pts.size() != target_pts.size()) {
    return;
  }

  source_points_ = source_pts;
  target_points_ = target_pts;
}

void CageDeformTool::Deform(const cv::Mat &src, cv::Mat *dst) {
  const int width = src.cols;
  const int height = src.rows;
  *dst = src.clone();
  // dst->setTo(cv::Scalar(255, 255, 255));

  cv::Rect img_rect(0, 0, width, height);
  // TODO: use triangulation
  // triangulation_.reset(new cv::Subdiv2D(img_rect));
  // triangulation_->insert(source_points_);

  // std::vector<int> triangles_indices;
  // std::vector<cv::Vec6f> triangles;
  // triangulation_->getTriangleList(triangles);
  // for (size_t i = 0; i < triangles.size(); ++i) {
  //   const auto &t = triangles[i];
  //   cv::Point pt_a(t[0], t[1]);
  //   cv::Point pt_b(t[2], t[3]);
  //   cv::Point pt_c(t[4], t[5]);
  //   if (!img_rect.contains(pt_a) || !img_rect.contains(pt_b) ||
  //       !img_rect.contains(pt_c)) {
  //     continue;
  //   }
  //   int i_a = GetSourcePointIndex(pt_a);
  //   int i_b = GetSourcePointIndex(pt_b);
  //   int i_c = GetSourcePointIndex(pt_c);
  //   triangles_indices.push_back(i_a);
  //   triangles_indices.push_back(i_b);
  //   triangles_indices.push_back(i_c);
  // }

  std::vector<std::vector<int> > paint_mask;
  // calc. barycentric coordinates
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      std::vector<float> bc_weights;
      CalcMeanValueBaryCentricCoordinates(cv::Point2f(j, i),
          &bc_weights);
      auto trans_pt = GetPointByBaryCentricCoordinates(
          cv::Point2f(j, i), bc_weights);
      if (!img_rect.contains(trans_pt)) {
        continue;
      }
      dst->at<cv::Vec3b>(trans_pt.y, trans_pt.x) =
          src.at<cv::Vec3b>(i, j);
    }
  }
}

int CageDeformTool::GetSourcePointIndex(const cv::Point2f &pt) {
  float min_dist = 1e9f;
  int min_dist_idx = -1;
  for (size_t i = 0; i < source_points_.size(); ++i) {
    const auto &src_pt = source_points_[i];
    float dist = std::hypot(std::fabs(pt.x - src_pt.x),
        std::fabs(pt.y - src_pt.y));
    if (dist < min_dist) {
      min_dist = dist;
      min_dist_idx = i;
    }
  }
  return min_dist_idx;
}

void CageDeformTool::CalcMeanValueBaryCentricCoordinates(
    const cv::Point2f &pt,
    std::vector<float> *weights) {
  weights->clear();
  float weight_sum = 0.f;
  const int num_poly_pts = source_points_.size();
  for (int i = 0; i < num_poly_pts; ++i) {
    const auto &cur_pt = source_points_[i];
    const auto &next_pt = source_points_[(i + 1) % num_poly_pts];
    const auto &prev_pt =
        source_points_[(num_poly_pts + i - 1) % num_poly_pts];
    float dist_to_pt = std::hypot(std::fabs(pt.x - cur_pt.x),
        std::fabs(pt.y - cur_pt.y));
    auto cur_v = cur_pt - pt;
    auto prev_v = prev_pt - pt;
    auto next_v = next_pt - pt;
    float prev_alpha = angleBetween(cur_v, prev_v);
    float next_alpha = angleBetween(cur_v, next_v);
    float weight = (std::tan(0.5f * prev_alpha) + std::tan(0.5f * next_alpha)) /
        dist_to_pt;
    weights->push_back(weight);
    weight_sum += weight;
  }
  for (size_t i = 0; i < weights->size(); ++i) {
    (*weights)[i] /= weight_sum;
  }
}

cv::Point2i CageDeformTool::GetPointByBaryCentricCoordinates(
    const cv::Point2f &pt, const std::vector<float> &weights) {
  float x = 0.f;
  float y = 0.f;
  for (size_t i = 0; i < weights.size(); ++i) {
    x += weights[i] * target_points_[i].x;
    y += weights[i] * target_points_[i].y;
  }
  return cv::Point2i(x, y);
}

}  // namespace cage_deform_tool

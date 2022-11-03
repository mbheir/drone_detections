#ifndef NMS_HPP__
#define NMS_HPP__

#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>

enum PontInRectangle {XMIN, YMIN, XMAX, YMAX};

std::vector<cv::Rect> nms(const std::vector<std:vector<float>> &, const float &);

std::vector<float> GetPointFromRect(const std::vector<std::vector<float>> &, const PointInRectangle &);

std::vector<float> ComputeArea(const std::vector<std::vector<float>> &, const std::vector<float> &, const std::vector<float> &, const std::vector<float> &);

template <typename T>
std::vector<int> argsort(const std::vector<T> & v);

std::vector<float> Maximum(const float &, const std::vector<float> &);

std::vector<float> Minimum(const float &, const std::vector<float> &);

std::vector<float> CopyByIndices(const std::vector<float> &, const std::vector<int> &);

std::vector<float> RemoveLast(const std::vector<int> &);

std::vector<float> Subtract(const std::vector<float> &, const std::vector<float> &);

std::vector<float> Multiply(const std::vector<float> &, const std::vector<float> &);

std::vector<float> Divide(const std::vector<float> &, const std::vector<float> &);

std::vector<float> WhereLarger(const std::vector<float> &, const float &);

std::vector<int> RemoveByIndices(const std::vector<int> &, const std::vector<int> &);

std::vector<cv::Rect> BoxesToRects(const std::vector<std::vector<float>> &);

template <typename T>
std::vector<T> FilterVector(const std::vector<T> &, const std::vector<int> &);

#endif // NMS_HPP__
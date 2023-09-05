#ifndef CONFIDENCEREGIONDEMO_H
#define CONFIDENCEREGIONDEMO_H

#include <filesystem>
#include "Camera.h"

void calibrationConfidenceRegionDemo(const Camera & cam, ChessboardData & chessboardData, const std::filesystem::path & outputDirectory, int interactive);

#endif
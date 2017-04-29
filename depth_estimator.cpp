#define _USE_MATH_DEFINES
#include <cmath>

#include "motion_estimator.hpp"
#include "depth_estimator.hpp"

DepthEstimator::DepthEstimator(int width, int height, uint8_t quality)
	: width(width)
	, height(height)
	, quality(quality)
	, width_ext(width + 2 * MotionEstimator::BORDER)
	, num_blocks_hor((width + MotionEstimator::BLOCK_SIZE - 1) / MotionEstimator::BLOCK_SIZE)
	, num_blocks_vert((height + MotionEstimator::BLOCK_SIZE - 1) / MotionEstimator::BLOCK_SIZE)
	, first_row_offset(width_ext * MotionEstimator::BORDER + MotionEstimator::BORDER) {
	// PUT YOUR CODE HERE
}

DepthEstimator::~DepthEstimator() {
	// PUT YOUR CODE HERE
}

void DepthEstimator::Estimate(const uint8_t* cur_Y,
                              const int16_t* cur_U,
                              const int16_t* cur_V,
                              const MV* mvectors,
                              uint8_t* depth_map) {
	// PUT YOUR CODE HERE
	constexpr double MULTIPLIER = 32.0;

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			const auto i = (y / MotionEstimator::BLOCK_SIZE);
			const auto j = (x / MotionEstimator::BLOCK_SIZE);
			auto mv = mvectors[i * num_blocks_hor + j];

			depth_map[y * width + x] = static_cast<uint8_t>(20 * std::sqrt(mv.x * mv.x + mv.y * mv.y));
		}
	}
	
	const int bord = g_size;
	uint8_t *depth_clone = new uint8_t[(width + 2 * g_size) * (height + 2 * g_size)];

	for (int y = 0; y < height + 2 * bord; ++y) {
		for (int x = 0; x < width + 2 * bord; ++x) {
			if (y < bord) {
				depth_clone[y * width + x] = depth_map[x];
			}
			else if (x < bord) {
				depth_clone[y * width + x] = depth_map[y * width];
			}
			else if (y >= height + bord) {
				depth_clone[y * width + x] = depth_map[x + width * (height - 1)];
			}
			else if (x >= width + bord) {
				depth_clone[y * width + x] = depth_map[y * width + width - 1];
			}
			else {
				depth_clone[y * width + x] = depth_map[(y - bord) * width + x - bord];
			}
		}
	}

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			double pix = 0, coef = 0;
			for (int i = -g_size / 2; i <= g_size / 2; i++) {
				for (int j = -g_size / 2; j <= g_size / 2; j++) {
					double cur_c = std::exp(-(i * i + j * j) / (2 * sig * sig)
						- (cur_Y[x + MotionEstimator::BORDER + (y + MotionEstimator::BORDER) * width_ext] - cur_Y[x + j + MotionEstimator::BORDER + (y + i + MotionEstimator::BORDER) * width_ext]) * (cur_Y[x + MotionEstimator::BORDER + (y + MotionEstimator::BORDER) * width_ext] - cur_Y[x + j + MotionEstimator::BORDER + (y + i + MotionEstimator::BORDER) * width_ext]) / (2 * sig_c * sig_c));
					cur_c /= 2 * M_PI * sig * sig_c;
					pix += depth_clone[x + j + bord + (y + i + bord) * width] * cur_c;
					coef += cur_c;
				}
			}
			depth_map[x + y * width] = uint8_t(pix / coef);
		}
	}
	delete[] depth_clone;
}

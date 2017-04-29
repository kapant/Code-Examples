#include <unordered_map>

#include "metric.hpp"
#include "motion_estimator.hpp"
#include <cmath>
#include <memory>
#include <algorithm>

MotionEstimator::MotionEstimator(int width, int height, uint8_t quality, bool use_half_pixel)
	: width(width)
	, height(height)
	, quality(quality)
	, use_half_pixel(use_half_pixel)
	, width_ext(width + 2 * BORDER)
	, num_blocks_hor((width + BLOCK_SIZE - 1) / BLOCK_SIZE)
	, num_blocks_vert((height + BLOCK_SIZE - 1) / BLOCK_SIZE)
	, first_row_offset(width_ext * BORDER + BORDER) {
	// PUT YOUR CODE HERE
}

MotionEstimator::~MotionEstimator() {
	// PUT YOUR CODE HERE
}

void MotionEstimator::Estimate8x8(const uint8_t* cur_Y,
	const uint8_t* prev_Y,
	const uint8_t* prev_Y_up,
	const uint8_t* prev_Y_left,
	const uint8_t* prev_Y_upleft,
	MV* mvectors) {
	std::unordered_map<ShiftDir, const uint8_t*> prev_map{
		{ ShiftDir::NONE, prev_Y }
	};

	if (use_half_pixel) {
		prev_map.emplace(ShiftDir::UP, prev_Y_up);
		prev_map.emplace(ShiftDir::LEFT, prev_Y_left);
		prev_map.emplace(ShiftDir::UPLEFT, prev_Y_upleft);
	}

	for (int i = 0; i < num_blocks_vert / 2; i++) {
		for (int j = 0; j < num_blocks_hor / 2; j++) {
			const auto block_id = i * num_blocks_hor / 2 + j;
			const auto hor_offset = 2 * j * BLOCK_SIZE;
			const auto vert_offset = first_row_offset + 2 * i * BLOCK_SIZE * width_ext;
			const auto cur = cur_Y + vert_offset + hor_offset;

			MV best_vector;
			best_vector.error = std::numeric_limits<long>::max();

			// PUT YOUR CODE HERE
			
			auto prev = prev_Y + vert_offset + hor_offset;

			for (int i1 = -1; i1 < 2; i1++) {
				for (int j1 = -1; j1 < 2; j1++) {
					cords[(i1 + 1) * 3 + j1 + 1].state = false;
					cords[(i1 + 1) * 3 + j1 + 1].x = 0;
					cords[(i1 + 1) * 3 + j1 + 1].y = 0;
					if (quality > 50 || (i1 == 0) || (i1 == -1 && j1 == 0) || (i1 == 1 && j1 == 0)) {
						if (i + i1 >= 0 && i + i1 < num_blocks_vert && j + j1 >= 0 && j + j1 < num_blocks_hor) {
							cords[(i1 + 1) * 3 + j1 + 1].x = mvectors[block_id + (i + i1) * num_blocks_hor + (j + j1)].x;
							cords[(i1 + 1) * 3 + j1 + 1].y = mvectors[block_id + (i + i1) * num_blocks_hor + (j + j1)].y;
							if (cords[(i1 + 1) * 3 + j1 + 1].y != 0 && cords[(i1 + 1) * 3 + j1 + 1].x != 0) {
								cords[(i1 + 1) * 3 + j1 + 1].state = false;
							}
						}
					}
				}
			}

			int dst = int(width / height * std::sqrt(100 / (quality + 10))) + 1;
			int dst_x = 0, dst_y = 0;

			for (int i1 = 0; i1 < 9; i1++) {
				if (cords[i1].state) {
					for (int j1 = i1 + 1; j1 < 9; j1++) {
						if (cords[j1].state) {
							if (std::abs(cords[i1].x - cords[j1].x) > dst_x) {
								dst_x = std::abs(cords[i1].x - cords[j1].x);
							}
							if (std::abs(cords[i1].y - cords[j1].y) > dst_y) {
								dst_y = std::abs(cords[i1].y - cords[j1].y);
							}
						}
					}
				}
			}

			int n = 0;
			for (int i1 = 0; i1 < 9; i1++) {
				if (cords[i1].state) {
					n++;
				}
			}

			best_vector.error = GetErrorSAD_8x8(cur, prev, width_ext);

			if (n != 0 && dst_x <= dst && dst_y <= dst) {
				int x = 0, y = 0;
				for (int i1 = 0; i1 < 9; i1++) {
					if (cords[i1].state) {
						x += cords[i1].x;
						y += cords[i1].y;
					}
				}
				best_vector.x = x / n;
				best_vector.y = y / n;
				best_vector.error = GetErrorSAD_8x8(cur, prev + best_vector.x + best_vector.y * width_ext, width_ext);
				prev += best_vector.x + best_vector.y * width_ext;
			}
			else if (n != 0) {
				for (int i1 = 0; i1 < 9; i1++) {
					if (cords[i1].state) {
						for (int j1 = i1 + 1; j1 < 9; j1++) {
							if (cords[j1].state && std::abs(cords[i1].x - cords[j1].x) + std::abs(cords[i1].y - cords[j1].y) < 2 * dst) {
								cords[j1].state = false;
							}
						}
					}
				}
				for (int i1 = 0; i1 < 9; i1++) {
					if (cords[i1].state) {
						auto er = GetErrorSAD_8x8(cur, prev + cords[i1].x + cords[i1].y * width_ext, width_ext);
						if (er < best_vector.error) {
							best_vector.x = cords[i1].x;
							best_vector.y = cords[i1].y;
							best_vector.error = er;
						}
					}
				}
				prev += best_vector.x + best_vector.y * width_ext;
			}
			else {
				int len = 6, mul = 1;
				if (quality < 50) {
					mul = 2;
				}
				for (int i1 = 0; i1 < 3; i1++) {
					int x1 = 0, y1 = 0;
					for (int x = -len; x <= len; x += mul * len / 2) {
						if (x != 0) {
							auto er = GetErrorSAD_8x8(cur, prev + x, width_ext);
							if (er < best_vector.error) {
								best_vector.error = er;
								x1 = x;
							}
						}
					}
					best_vector.x += x1;
					prev += x1;
					for (int y = -int(dst); y <= int(dst); y += 2) {
						if (y != 0) {
							auto er = GetErrorSAD_8x8(cur, prev + y * width_ext, width_ext);
							if (er < best_vector.error) {
								best_vector.error = er;
								y1 = y;
							}
						}
					}
					best_vector.y += y1;
					prev += y1 * width_ext;
					len /= dst;
				}
			}

			int x = 0, y = 0;
			for (int i1 = -1; i1 < 2; i1++) {
				for (int j1 = -1; j1 < 2; j1++) {
					if (quality > 50 || (i1 == 0) || (i1 == -1 && j1 == 0) || (i1 == 1 && j1 == 0)) {
						auto er = GetErrorSAD_8x8(cur, prev + i1 * width_ext + j1, width_ext);
						if (er < best_vector.error) {
							best_vector.error = er;
							x = j1;
							y = i1;
						}
					}
				}
			}
			best_vector.x += x;
			best_vector.y += y;

			mvectors[block_id] = best_vector;
		}
	}
}

uint8_t* MotionEstimator::ShiftDown(const uint8_t* cur_Y) {
	uint8_t *out = new uint8_t[width_ext * (height + 2 * BORDER)];

	//UP BORDER
	for (int i = 0; i < BORDER; i++) {
		for (int j = 0; j < width_ext; j++) {
			out[j + i * width_ext] = cur_Y[j + i * width_ext];
		}
	}

	for (int i = BORDER; i < height + 2 * BORDER - 1; i++) {
		for (int j = 0; j < width_ext; j++) {
			if (i < BORDER + 4 ) {
				out[j + i * width_ext] = cur_Y[BORDER * width_ext + j];
			}
			else {
				out[j + i * width_ext] = cur_Y[(i - 4) * width_ext + j];
			}
		}
	}
	return out;
}

uint8_t* MotionEstimator::ShiftRight(const uint8_t* cur_Y) {
	uint8_t *out = new uint8_t[width_ext * (height + 2 * BORDER)];

	//RIGHT BORDER
	for (int j = 0; j < BORDER; j++) {
		for (int i = 0; i < height + 2 * BORDER - 1; i++) {
			out[j + i * width_ext] = cur_Y[j + i * width_ext];
		}
	}

	for (int j = BORDER; j < width_ext; j++) {
		for (int i = 0; i < height + 2 * BORDER - 1; i++) {
			if (j < BORDER + 4) {
				out[j + i * width_ext] = cur_Y[i * width_ext + BORDER];
			}
			else {
				out[j + i * width_ext] = cur_Y[i * width_ext + j - 4];
			}
		}
	}

	return out;
}

void MotionEstimator::Estimate(const uint8_t* cur_Y,
                               const uint8_t* prev_Y,
                               const uint8_t* prev_Y_up,
                               const uint8_t* prev_Y_left,
                               const uint8_t* prev_Y_upleft,
                               MV* mvectors) {
	uint8_t *cur, *prev;
	MV *temp_vectors = new MV[num_blocks_hor * num_blocks_vert];
	Estimate8x8(cur_Y, prev_Y, prev_Y_up, prev_Y_left, prev_Y_upleft, temp_vectors);
	
	for (int i = 0; i < num_blocks_vert / 2; i++) {
		for (int j = 0; j < num_blocks_hor / 2; j++) {
			mvectors[2 * i * num_blocks_hor + 2 * j] = temp_vectors[i * num_blocks_hor / 2 + j];
		}
	}

	cur = ShiftDown(cur_Y);
	prev = ShiftDown(prev_Y);
	Estimate8x8(cur, prev, prev_Y_up, prev_Y_left, prev_Y_upleft, temp_vectors);
	for (int i = 0; i < num_blocks_vert / 2; i++) {
		for (int j = 0; j < num_blocks_hor / 2; j++) {
			mvectors[(2 * i  + 1) * num_blocks_hor + 2 * j] = temp_vectors[i * num_blocks_hor / 2 + j];
		}
	}
	delete[] cur;
	delete[] prev;

	cur = ShiftRight(cur_Y);
	prev = ShiftRight(prev_Y);
	Estimate8x8(cur, prev, prev_Y_up, prev_Y_left, prev_Y_upleft, temp_vectors);
	for (int i = 0; i < num_blocks_vert / 2; i++) {
		for (int j = 0; j < num_blocks_hor / 2; j++) {
			mvectors[2 * i * num_blocks_hor + 2 * j + 1] = temp_vectors[i * num_blocks_hor / 2 + j];
		}
	}
	delete[] cur;
	delete[] prev;

	uint8_t *cur1 = ShiftRight(cur_Y);
	uint8_t *prev1 = ShiftRight(prev_Y);
	cur = ShiftDown(cur1);
	prev = ShiftDown(prev1);
	delete[] cur1;
	delete[] prev1;
	Estimate8x8(cur, prev, prev_Y_up, prev_Y_left, prev_Y_upleft, temp_vectors);
	for (int i = 0; i < num_blocks_vert / 2; i++) {
		for (int j = 0; j < num_blocks_hor / 2; j++) {
			mvectors[(2 * i + 1) * num_blocks_hor + 2 * j + 1] = temp_vectors[i * num_blocks_hor / 2 + j];
		}
	}
	delete[] cur;
	delete[] prev;

	delete[] temp_vectors;
}

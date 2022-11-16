#include "pso_slam/frame.h"
#include "pso_slam/core.h"
#include <cstdio>

#ifdef OPENCV_FOUND
#include <cmath>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#endif

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif

PSOFrame::PSOFrame(Vector3d trans, unsigned short width, unsigned short height, double cell_side, bool init_cell_windows, PSOConfig config
#if BUILD_OCCUPANCY_GRID
                   ,
                   double occupancy_grid_cell_size
#endif
                   )
    : s_trans(std::move(trans)), s_config(std::move(config)), width(width), height(height), cell_side(cell_side) {
  this->built = false;
  this->widthNumOfCells = uint16_t(ceil(width / cell_side));
  this->heightNumOfCells = uint16_t(ceil(height / cell_side));
  this->numOfCells = widthNumOfCells * heightNumOfCells;
  this->cells = vector<PSOCell>(this->numOfCells, PSOCell(init_cell_windows));

#if BUILD_OCCUPANCY_GRID
  // Initializing the occupancy grid,
  this->s_occupancy_grid.cell_size = occupancy_grid_cell_size;

  // I case of zero size cell, all operations on the occupancy grid are ommited,
  // This saves some computing time when OG aren't needed (like in intermediate
  // frames used for loading laser data and matching)
  if (occupancy_grid_cell_size > 0.) {
    this->s_occupancy_grid.width = uint32_t(ceil(width / occupancy_grid_cell_size));
    this->s_occupancy_grid.height = uint32_t(ceil(height / occupancy_grid_cell_size));
    this->s_occupancy_grid.count = this->s_occupancy_grid.width * this->s_occupancy_grid.height;
    this->s_occupancy_grid.og = vector<int8_t>(this->s_occupancy_grid.count, 0);
  }
#endif

#if false
    /*
    TODO: Needs to be generic !!
    */

    if (positive_only) {
        this->s_x_min = -7.2; // <-- here
        this->s_x_max = width - 8.2; // <-- and here!
    } else {
#endif
  this->s_x_min = -width / 2.;
  this->s_x_max = width / 2.;

#if false
    }
#endif

  this->s_y_min = -height / 2.;
  this->s_y_max = height / 2.;
}

void PSOFrame::build() {
#if BUILD_OCCUPANCY_GRID
  auto og_cells_per_cell = static_cast<uint32_t>(floor(this->cell_side / this->s_occupancy_grid.cell_size));
#endif

  for (unsigned int i = 0; i < this->numOfCells; ++i) {
    auto current_cell = &this->cells[i];

    if (current_cell->created) {
      current_cell->build();

#if BUILD_OCCUPANCY_GRID
      if (this->s_occupancy_grid.cell_size > 0.) {
        uint32_t cell_x_ind = i % this->widthNumOfCells, cell_y_ind = i / this->heightNumOfCells;

        for (uint32_t j = 0; j < og_cells_per_cell; ++j) {
          for (uint32_t k = 0; k < og_cells_per_cell; ++k) {
            double x_c = ((cell_x_ind * og_cells_per_cell + j) * this->s_occupancy_grid.cell_size + this->s_occupancy_grid.cell_size / 2.) -
                         (this->width / 2.);

            double y_c = ((cell_y_ind * og_cells_per_cell + k) * this->s_occupancy_grid.cell_size + this->s_occupancy_grid.cell_size / 2.) -
                         (this->height / 2.);

            auto p = current_cell->normalDistribution(Vector2d(x_c, y_c)) /* - 0.5*/;

            if (p > 0.) {
              uint32_t og_x_ind = cell_x_ind * og_cells_per_cell + j;
              uint32_t og_y_ind = cell_y_ind * og_cells_per_cell + k;

              /*
               * TODO: move this outside to store the max/min x and y, as
               * coordinates not as indexes, so the same info can be used with
               * the point cloud map
               */
              this->s_occupancy_grid.min_x_ind = MIN(og_x_ind, this->s_occupancy_grid.min_x_ind);
              this->s_occupancy_grid.max_x_ind = MAX(og_x_ind, this->s_occupancy_grid.max_x_ind);
              this->s_occupancy_grid.min_y_ind = MIN(og_y_ind, this->s_occupancy_grid.min_y_ind);
              this->s_occupancy_grid.max_y_ind = MAX(og_y_ind, this->s_occupancy_grid.max_y_ind);

              this->s_occupancy_grid.og[og_x_ind + this->s_occupancy_grid.height * og_y_ind] = int8_t(p * 100.);
            }
          }
        }
      }
#endif
    }
  }

  this->built = true;
}

void PSOFrame::transform(Vector3d trans) {
  if (!trans.isZero(1e-6)) {
    vector<PSOCell>* old_cells = &this->cells;

    this->cells = vector<PSOCell>(this->numOfCells);

    // for (unsigned int i = 0; i < this->numOfCells; ++i)
    for (auto& old_cell : (*old_cells)) {
      if (old_cell.created) {
        for (auto& points_vec : old_cell.points_vector) {
          for (auto& point : points_vec) {
            Vector2d new_point = transform_point(point, trans);
            this->addPoint(new_point);
          }
        }
      }
    }

    delete old_cells;
    this->built = false;
  }
}

// Initialize the cell from laser data according to the device sensibility and
// the minimum angle
void PSOFrame::loadLaser(vector<float> const& laser_data, float const& min_angle, float const& angle_increment, float const& max_range) {
  this->built = false;
  auto n = static_cast<unsigned int>(laser_data.size());

#if TRANSFORM_POINTS_AT_LOAD
  // Define a function 'f' to do transformation if needed
  Vector2d (*trans_func)(const Vector2d&, const Vector3d&) = nullptr;

  if (!this->s_trans.isZero(1e-6))
    trans_func = &transform_point;
#endif

  float theta;
#if PREFER_FRONTAL_POINTS
  float delta_theta = 0.f;
#endif

  // For each element in the laser vector, get his index (i) and it's
  // corresponding (theta)
  // according to the sensibility and the minimum angle
  for (unsigned int i = 0; i < n; ++i) {
    if ((laser_data[i] > 0.) && (laser_data[i] < max_range) && (laser_data[i] > this->s_config.laserIgnoreEpsilon)) {
      theta = index_to_angle(i, angle_increment, min_angle);
#if PREFER_FRONTAL_POINTS
      delta_theta += cosf(theta);

      if (fabsf(delta_theta) > .5f) {
#endif
        Vector2d point = laser_to_point(laser_data[i], theta);

#if TRANSFORM_POINTS_AT_LOAD
        if (trans_func)
          point = trans_func(point, this->s_trans);
#endif
        this->addPoint(point);
#if PREFER_FRONTAL_POINTS
        delta_theta = 0.f;
      }
#endif
    }
  }
}

void PSOFrame::update(Vector3d trans, PSOFrame* const new_frame) {
  this->built = false; // Set 'built' flag to false to rebuild the cell if needed

  for (auto& new_frame_cell : new_frame->cells) {
    if (new_frame_cell.created) {
      for (auto& point : new_frame_cell.points_vector[0]) {
        Vector2d pt = transform_point(point, trans);
        this->addPoint(pt);
      }
    }
  }
}

void PSOFrame::addPose(double timestamp, const Vector3d& pose, const Vector3d& odom) {
  // Used only for saving the global map image (if any), for scan matching;
  // there is no need for this. Useful for debug
  this->s_timestamps.push_back(timestamp);
  this->s_poses.push_back(pose);
  this->s_odoms.push_back(odom);
}

void PSOFrame::resetCells() {
  auto n = static_cast<unsigned int>(this->cells.size());
  for (unsigned int i = 0; i < n; ++i)
    this->cells[i].reset();
}

// Add the given point 'pt' to it's corresponding cell
void PSOFrame::addPoint(Vector2d& point) {
  // Get the cell index in the list
  int cell_index = this->getCellIndex(point, this->widthNumOfCells, this->cell_side);

  // If the point is contained in the frame borders and it's not at the origin
  if (-1 != cell_index) {
    // And then, append the point to its cell points list
    this->cells[static_cast<size_t>(cell_index)].addPoint(point);

    this->built = false; // Set 'built' flag to false to rebuild the cell if needed
  }

#if BUILD_OCCUPANCY_GRID && false
  if (this->s_occupancy_grid.cell_size > 0.) {
    cell_index = this->getCellIndex(point, static_cast<int>(this->s_occupancy_grid.width), this->s_occupancy_grid.cell_size);
    if (-1 != cell_index) {
      this->s_occupancy_grid.og[static_cast<unsigned int>(cell_index)]++;
    }
  }
#endif
}

// volatile const char *(*signal(int const * b, void (*fp)(int*)))(int**); //
// Just for fun!

int PSOFrame::getCellIndex(Vector2d point, int grid_width, double cell_side) {
  // If the point is contained inside the FRAME borders
  if ((point.x() > this->s_x_min) && (point.x() < this->s_x_max) && (point.y() > this->s_y_min) && (point.y() < this->s_y_max)) {
    // Then return the index of the its corresponding CELL
    return static_cast<int>(floor((point.x() + (this->width / 2.)) / cell_side) +
                            grid_width * (floor((point.y() + (this->height / 2.)) / cell_side)));
  }

  return -1;
}

Vector3d PSOFrame::align(Vector3d initial_guess, const PSOFrame* const new_frame) {
  // Used to UNIFORMLY distribute the initial particles
  Vector3d deviation = this->s_iter < 2 ? Vector3d(.1, .1, 3.1415E-3) : (this->s_pose_diff * 2.).array().abs();

  ++this->s_iter;

  auto pose = pso_optimization(std::move(initial_guess), this, new_frame, std::move(deviation));

#if TRANSFORM_POSE_AFTER_ALIGN
  pose -= this->s_trans;
#endif

  this->s_pose_diff = pose - this->s_prev_pose;
  this->s_prev_pose = pose;
  return pose;
}

void PSOFrame::dumpMap(const char* filename, bool save_poses, bool save_points, bool save_image, short density
#if BUILD_OCCUPANCY_GRID
                       ,
                       bool save_occupancy_grid
#endif
) {
  // Save the points, poses & odoms to an image, useful for debugging!
  FILE *hndl_poses = nullptr, *hndl_points = nullptr;
  char output_filename[250];

  if (save_poses) {
    sprintf(output_filename, "%s.pose.csv", filename);
    hndl_poses = fopen(output_filename, "w");
    if (hndl_poses)
      fprintf(hndl_poses, "timestamp,xP,yP,thP,xO,yO,thO\n");
  }

  if (save_points) {
    sprintf(output_filename, "%s.map.csv", filename);
    hndl_points = fopen(output_filename, "w");
    if (hndl_points)
      fprintf(hndl_points, "x,y\n");
  }

  if ((save_poses && !hndl_poses) || (save_points && !hndl_points)) {
    printf("%s: Cannot open files, cannot save!\n ", __func__);
    return;
  }

#ifdef OPENCV_FOUND
  int size_x = this->width * density, // density in "pixel per meter"
      size_y = this->height * density;

  int counter = 0;

  cv::Mat img(size_x, size_y, CV_8UC3, cv::Scalar::all(255));

  // Draw a grid (using `density` as increment, we draw a line each 1 meter)
  for (int i = 0; i < size_x; i += density) {
    cv::line(img, cv::Point(i, 0), cv::Point(i, size_y), cv::Scalar(180, 180, 180));
    cv::line(img, cv::Point(0, i), cv::Point(size_x, i), cv::Scalar(180, 180, 180));
  }
#endif

  // Draw and dump 2D points
  for (auto& cell : this->cells) {
    for (auto& points_vec : cell.points_vector) {
      for (auto& point : points_vec) {
        // for (unsigned int j = 0; j < points.size(); ++j) {
#ifdef OPENCV_FOUND
        int x = (size_x / 2) + static_cast<int>(point.x() * density);
        int y = (size_y / 2) - static_cast<int>(point.y() * density);
        cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0));
#endif
        if (save_points) {
          fprintf(hndl_points, "%.5f,%.5f\n", point.x(), point.y());
        }
      }
    }
  }

  // Draw and dump poses
  for (unsigned int i = 0; i < this->s_poses.size(); ++i) {
#ifdef OPENCV_FOUND
    auto x = (size_x / 2) + static_cast<int>(this->s_poses[i].x() * density);
    auto y = (size_y / 2) - static_cast<int>(this->s_poses[i].y() * density);
    auto dx = static_cast<int>(.5 * cos(-this->s_poses[i].z()) * density);
    auto dy = static_cast<int>(.5 * sin(-this->s_poses[i].z()) * density);

    if (0 == counter) {
      cv::line(img, cv::Point(x, y), cv::Point(x + dx, y + dy), cv::Scalar(40, 40, 80));
    }

    cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0, 0, 255));

    counter = (counter + 1) % 5;
#endif
    if (save_points) {
      fprintf(hndl_poses, "%.6f,%.5f,%.5f,%.5f\n", this->s_timestamps[i], this->s_poses[i].x(), this->s_poses[i].y(), this->s_poses[i].z());
    }
  }

  if (save_poses)
    fclose(hndl_poses);

  if (save_points)
    fclose(hndl_points);

  if (save_poses || save_points) {
    // Save the .gnuplot file to plot the outputs
    sprintf(output_filename, "%s.gnuplot", filename);
    hndl_poses = fopen(output_filename, "w");

    fprintf(hndl_poses, "set datafile separator ','\n"
                        "set key autotitle columnhead\n"
                        "set size ratio -1\n"
                        "plot ");

    if (save_points) {
      fprintf(hndl_poses,
              "'%s.map.csv' title 'Map' with points "
              "pointsize 0.2 "
              "pointtype 5 linecolor rgb '#555555'",
              filename);
    }

    if (save_poses) {
      fprintf(hndl_poses,
              ", \\"
              "\n"
              "'%s.pose.csv' using 2:3 title 'Pose (LiDAR)' with "
              "linespoints linewidth 0.7 "
              "pointtype 6 pointsize 0.7 linecolor rgb '#ff0000'",
              filename);
    }

    fprintf(hndl_poses, "\n"
                        "pause 1000\n");

    fclose(hndl_poses);
  }

#ifdef OPENCV_FOUND
  if (save_image) {
    sprintf(output_filename, "%s-w%d-%dp%di-%dx%d-c%.2f-%dppm.png", filename, PSO_WINDOW_SIZE, this->s_config.psoConfig.populationSize,
            this->s_config.psoConfig.iterations, this->width, this->height, this->cell_side, density);

    imwrite(output_filename, img);
  }

#if BUILD_OCCUPANCY_GRID
  if (save_occupancy_grid) {
    uint32_t real_width = this->s_occupancy_grid.max_x_ind - this->s_occupancy_grid.min_x_ind,
             real_heigth = this->s_occupancy_grid.max_y_ind - this->s_occupancy_grid.min_y_ind;

    cv::Mat img_og(static_cast<int>(real_heigth), static_cast<int>(real_width), CV_8U, cv::Scalar::all(255));

    for (uint32_t i = this->s_occupancy_grid.min_x_ind; i <= this->s_occupancy_grid.max_x_ind; ++i) {
      for (uint32_t j = this->s_occupancy_grid.min_y_ind; j <= this->s_occupancy_grid.max_y_ind; ++j) {
        size_t ind = i + this->s_occupancy_grid.height * j;
        if (this->s_occupancy_grid.og[ind] > 0) {
          img_og.at<uint8_t>(int(real_heigth - (j - this->s_occupancy_grid.min_y_ind)), int(i - this->s_occupancy_grid.min_x_ind)) =
              uint8_t(255.0 - this->s_occupancy_grid.og[i + this->s_occupancy_grid.height * j] * 2.55);
        }
      }
    }

    sprintf(output_filename, "%s-%dx%d-cell%.2fm-occupancy-grid.png", filename, this->s_occupancy_grid.width, this->s_occupancy_grid.height,
            this->s_occupancy_grid.cell_size);

    imwrite(output_filename, img_og);
  }
#endif
#endif
}

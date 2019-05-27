#include <gtest/gtest.h>

#include "../include/landing_site_detection/landing_site_detection.hpp"
#include "../include/landing_site_detection/landing_site_detection_node.hpp"


#include <chrono>
#include <iostream>
#include <random>
#include <numeric>
#include <fstream>
#include <sstream>
#include <vector>

using namespace landing_site_detection;

class TestLandingSite : public :: LandingSiteDetection {
public:
  Grid& test_getGrid() { return grid_;}
  float& test_getCounterThreshold() {return n_points_thr_;}
};

class LandingSiteDetectionTests : public ::testing::Test {
 public:
  TestLandingSite landing_site_detection;
  Eigen::Quaternionf q;

  void SetUp() override {
    ros::Time::init();
    Eigen::Vector3f pos(4.2f, 3.9f, 5.f);
    q = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);
    landing_site_detection.setPose(pos, q);

  }
  void TearDown() override {}
};

TEST_F(LandingSiteDetectionTests, flat_center) {
  LandingSiteDetectionNodeConfig config = LandingSiteDetectionNodeConfig::__getDefault__();

  config.smoothing_size = -1;
  config.min_n_land_cells = 0;
  config.n_points_threshold = 20;
  config.cell_size = 1;

  landing_site_detection.dynamicReconfigureSetParams(config, 1);

  // create pointcloud with 3x3m flat area in the center
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<float> distribution(0.0f, 1.0f);

  Eigen::Vector3f pos(5.f, 5.f, 5.f);
  landing_site_detection.setPose(pos, q);

  for (int i = 0; i < 1000; ++i) {
    float min = 0.f;
    float max = 9.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    min = 0.f;
    max = 3.f;
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  for (int i = 0; i < 1000; ++i) {
    float min = 0.f;
    float max = 9.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    min = 6.f;
    max = 9.f;
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  for (int i = 0; i < 500; ++i) {
    float min = 0.f;
    float max = 3.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    min = 3.f;
    max = 6.f;
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  for (int i = 0; i < 500; ++i) {
    float min = 6.f;
    float max = 9.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    min = 3.f;
    max = 6.f;
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  std::normal_distribution<float> distribution_flat(0.0f, 0.1f);
  for (int i = 0; i < 500; ++i) {
    float min = 3.f;
    float max = 6.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat(generator)));
  }

  landing_site_detection.runLandingSiteDetection();

  for (size_t i = 0; i < 10; i++) {
    for (size_t j = 0; j < 10; j++) {
      if (i >= 3 && i <= 5 && j >=3 && j <=5) {
        ASSERT_TRUE(landing_site_detection.grid_.land_(i, j));
      } else {
        ASSERT_FALSE(landing_site_detection.grid_.land_(i, j));
      }
    }
  }
}

TEST_F(LandingSiteDetectionTests, flat_bottom_half) {
  LandingSiteDetectionNodeConfig config = LandingSiteDetectionNodeConfig::__getDefault__();

  config.smoothing_size = -2;
  config.min_n_land_cells = 0;
  config.n_points_threshold = 4;
  config.cell_size = 1;

  landing_site_detection.dynamicReconfigureSetParams(config, 1);

  // create pointcloud with 3x3m flat area on the right side
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<float> distribution(2.0f, 1.5f);

  for (int i = 0; i < 1000; ++i) {
    float min_x = -0.8f;
    float max_x = 4.2f;
    float x = min_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_x - min_x)));
    float min_y = -1.1f;
    float max_y = 8.9f;
    float y = min_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_y - min_y)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  std::normal_distribution<float> distribution_flat(0.0f, 0.05f);
  for (int i = 0; i < 1000; ++i) {
    float min_x = 4.2f;
    float max_x = 9.2;
    float x = min_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_x - min_x)));
    float min_y = -1.1f;
    float max_y = 8.9f;
    float y = min_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_y - min_y)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat(generator)));
  }

  landing_site_detection.runLandingSiteDetection();

  for (size_t i = 0; i < 10; i++) {
    for (size_t j = 0; j < 5; j++) {
      ASSERT_FALSE(landing_site_detection.grid_.land_(j, i));
    }

    for (size_t j = 5; j < 10; j++) {
      ASSERT_TRUE(landing_site_detection.grid_.land_(j, i));
    }
  }
}

TEST_F(LandingSiteDetectionTests, binning_10_1) {

  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-4.6f, -2.1f, 1.213f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(15.2f, -13.4f, 0.987f));

  landing_site_detection.cloud_.push_back(pcl::PointXYZ(4.2f, 3.9f, 1.145f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-0.5f, -1.f, 1.198f)); //index (0,0)
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(8.9f, 8.6f, 1.065f)); //index (9,9)

  landing_site_detection.cloud_.push_back(pcl::PointXYZ(4.2f, 2.f, 1.287f)); //index (5, 3)

  landing_site_detection.cloud_.push_back(pcl::PointXYZ(9.1f, 8.5f, 1.047f )); //index (9,9)
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.134f)); //index (9,9)
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.144f)); //index (9,9)
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.104f)); //index (9,9)
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.110f)); //index (9,9)

  landing_site_detection.runLandingSiteDetection();
  Eigen::Vector2i p = Eigen::Vector2i(0, 0);
  Eigen::Vector2i p1 = Eigen::Vector2i(9, 9);
  Eigen::Vector2i p2 = Eigen::Vector2i(5, 3);
  Eigen::Vector2i p3 = Eigen::Vector2i(5, 5);
  EXPECT_FLOAT_EQ(1.198f, landing_site_detection.test_getGrid().getMean(p));
  EXPECT_FLOAT_EQ((1.065f + 1.047f + 1.134f + 1.144f + 1.104f + 1.110f) / 6.f, landing_site_detection.test_getGrid().getMean(p1));
  EXPECT_FLOAT_EQ(1.287f, landing_site_detection.test_getGrid().getMean(p2));
  EXPECT_FLOAT_EQ(1.145f, landing_site_detection.test_getGrid().getMean(p3));

  std::vector<float> v = {1.065f, 1.047f, 1.134f, 1.144f, 1.104f, 1.110f};
  float sum = std::accumulate(v.begin(), v.end(), 0.0);
  float mean = sum / v.size();
  float sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
  float variance = (sq_sum / v.size() - mean * mean);
  ASSERT_NEAR(variance, landing_site_detection.test_getGrid().getVariance(p1), 0.0001f);
}

TEST_F(LandingSiteDetectionTests, binning_12_4) {

  Eigen::Vector3f pos(-4.3f, 16.2f, 5.f);
  landing_site_detection.setPose(pos, q);

  LandingSiteDetectionNodeConfig config = LandingSiteDetectionNodeConfig::__getDefault__();
  config.grid_size = 12.f;
  config.cell_size = 4.f;
  landing_site_detection.dynamicReconfigureSetParams(config, 1);

  Eigen::Vector2i p0 = Eigen::Vector2i(0, 0);
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-10.289, 10.25, 0.023f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-10.2, 11.2, 0.043f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-6.33, 11.2, 0.086f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-10.29, 11.2, 0.073f));

  Eigen::Vector2i p1 = Eigen::Vector2i(1, 0);
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-6.01f, 12.3f, 0.036f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-2.32f, 14.f, 0.045f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-6.29f, 14.19f, 0.025f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-4.59f, 10.21f, 0.085f));

  Eigen::Vector2i p2 = Eigen::Vector2i(2, 0);
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(1.69f, 12.3f, 0.036f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 1.045f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 0.333f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 1.995f));
  landing_site_detection.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 0.245f));

  landing_site_detection.runLandingSiteDetection();

  std::vector<float> v0 = {0.023f, 0.043f, 0.086f, 0.073f};
  float sum = std::accumulate(v0.begin(), v0.end(), 0.0f);
  float mean = sum / v0.size();
  float sq_sum = std::inner_product(v0.begin(), v0.end(), v0.begin(), 0.0f);
  float variance = (sq_sum / v0.size() - mean * mean);

  EXPECT_FLOAT_EQ(mean, landing_site_detection.test_getGrid().getMean(p0));
  EXPECT_FLOAT_EQ(variance, landing_site_detection.test_getGrid().getVariance(p0));

  std::vector<float> v1 = {0.036f, 0.045f, 0.025f, 0.085f};
  sum = std::accumulate(v1.begin(), v1.end(), 0.0f);
  mean = sum / v1.size();
  sq_sum = std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0f);
  variance = (sq_sum / v1.size() - mean * mean);
  EXPECT_FLOAT_EQ(mean, landing_site_detection.test_getGrid().getMean(p1));
  EXPECT_FLOAT_EQ(variance, landing_site_detection.test_getGrid().getVariance(p1));


  std::vector<float> v2 = {0.036f, 1.045f, 0.333f, 1.995f, 0.245f};
  sum = std::accumulate(v2.begin(), v2.end(), 0.0f);
  mean = sum / v2.size();
  sq_sum = std::inner_product(v2.begin(), v2.end(), v2.begin(), 0.0f);
  variance = (sq_sum / v2.size() - mean * mean);
  EXPECT_FLOAT_EQ(mean, landing_site_detection.test_getGrid().getMean(p2));
  EXPECT_FLOAT_EQ(variance, landing_site_detection.test_getGrid().getVariance(p2));

}

TEST_F(LandingSiteDetectionTests, smoothing) {

  landing_site_detection.grid_.land_.fill(0);
  LandingSiteDetectionNodeConfig config = LandingSiteDetectionNodeConfig::__getDefault__();

  config.smoothing_size = 2;
  config.min_n_land_cells = 2;
  config.n_points_threshold = 0;
  config.cell_size = 1;

  landing_site_detection.dynamicReconfigureSetParams(config, 1);
  landing_site_detection.runLandingSiteDetection();

  landing_site_detection.grid_.reset();

  for (int i = 0; i < landing_site_detection.grid_.land_.rows(); i++) {
    for (int j = 0; j < landing_site_detection.grid_.land_.cols(); j++) {
      Eigen::Vector2i p(i, j);
      landing_site_detection.grid_.setVariance(p, 1.f);
      landing_site_detection.grid_.increaseCounter(p);
    }
  }

  Eigen::Vector2i p(2,3);
  landing_site_detection.grid_.setVariance(p, 0.001f);
  landing_site_detection.grid_.increaseCounter(p);
  Eigen::Vector2i p1(2,4);
  landing_site_detection.grid_.setVariance(p1, 0.001f);
  landing_site_detection.grid_.increaseCounter(p1);
  Eigen::Vector2i p2(2,5);
  landing_site_detection.grid_.setVariance(p2, 0.001f);
  landing_site_detection.grid_.increaseCounter(p2);
  Eigen::Vector2i p3(9,9);
  landing_site_detection.grid_.setVariance(p3, 0.001f);
  landing_site_detection.grid_.increaseCounter(p3);

  landing_site_detection.isLandingPossible();

  for (int i = 0; i < landing_site_detection.grid_.land_.rows(); i++) {
    for (int j = 0; j < landing_site_detection.grid_.land_.cols(); j++) {
      if (i >= 3 && i <=5 && j >= 0 && j <=4) {
        ASSERT_TRUE(landing_site_detection.grid_.land_(j, i));
      } else {
        ASSERT_FALSE(landing_site_detection.grid_.land_(j, i));
      }
    }
  }
}

TEST_F(LandingSiteDetectionTests, mean) {

  Eigen::Vector3f pos(5.f, 5.f, 5.f);
  landing_site_detection.setPose(pos, q);

  LandingSiteDetectionNodeConfig config = LandingSiteDetectionNodeConfig::__getDefault__();

  config.smoothing_size = 1;
  config.min_n_land_cells = 0;
  config.n_points_threshold = 1;
  config.cell_size = 1;
  config.max_n_mean_diff_cells = 1;

  landing_site_detection.dynamicReconfigureSetParams(config, 1);

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<float> distribution_flat_ground(0.0f, 0.05f);
  for (int i = 0; i < 2000; ++i) {
    float min = 0.f;
    float max = 10.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat_ground(generator)));
  }

  std::normal_distribution<float> distribution_flat_box(1.5f, 0.05f);
  for (int i = 0; i < 500; ++i) {
    float min = 4.f;
    float max = 5.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat_box(generator)));
  }

  landing_site_detection.runLandingSiteDetection();

  for (int i = 0; i < landing_site_detection.grid_.land_.rows(); i++) {
    for (int j = 0; j < landing_site_detection.grid_.land_.cols(); j++) {
      if (i == 4 && j == 4) {
        ASSERT_FALSE(landing_site_detection.grid_.land_(j, i));
      } else {
        ASSERT_TRUE(landing_site_detection.grid_.land_(j, i));
      }
    }
  }
}

TEST_F(LandingSiteDetectionTests, test_mean_variance) {

  Eigen::Vector3f pos(-4.3f, 16.2f, 5.f);
  landing_site_detection.setPose(pos, q);

  LandingSiteDetectionNodeConfig config = LandingSiteDetectionNodeConfig::__getDefault__();
  config.grid_size = 12.f;
  config.cell_size = 4.f;
  landing_site_detection.dynamicReconfigureSetParams(config, 1);
  std::vector<float> v0;
  Eigen::Vector2i p0 = Eigen::Vector2i(0, 0);
  for (int i = 0; i < 1000; ++i) {
    float min = 4.f;
    float max = 10.f;
    float z = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    v0.push_back(z);
    landing_site_detection.cloud_.push_back(pcl::PointXYZ(-10.289, 10.25, z));
  }

  landing_site_detection.runLandingSiteDetection();

  float sum = std::accumulate(v0.begin(), v0.end(), 0.0f);
  float mean = sum / v0.size();
  float sq_sum = std::inner_product(v0.begin(), v0.end(), v0.begin(), 0.0f);
  float variance = (sq_sum / v0.size() - mean * mean);

  ASSERT_NEAR(mean,  landing_site_detection.test_getGrid().getMean(p0), 0.001f);
  ASSERT_NEAR(variance,  landing_site_detection.test_getGrid().getVariance(p0), 0.001f);

}

TEST_F(LandingSiteDetectionTests, test_cloud_4) {

  Eigen::Vector3f pos(-5.2, 2.35, 11.9);

  landing_site_detection.setPose(pos, q);

  LandingSiteDetectionNodeConfig config = LandingSiteDetectionNodeConfig::__getDefault__();
  config.mean_diff_thr = 0.1f;
  config.n_points_threshold = 10;
  config.smoothing_size = -1;
  landing_site_detection.dynamicReconfigureSetParams(config, 1);

  std::string line;
  std::ifstream file_read ("/home/martina/Documents/cloud_4.txt");
  std::vector<double> p(3);

  if (file_read.is_open()) {
    while ( getline (file_read,line) ) {
      std::stringstream ss( line );
      std::string field;
      for (int i=0; i<3; ++i) {
        std::getline( ss, field, ',' );

        std::stringstream fs( field );
        double f = 0.0;  // (default value is 0.0)
        fs >> f;
        p[i] = f;
      }

      landing_site_detection.cloud_.push_back(pcl::PointXYZ(p[0], p[1], p[2]));
      p.clear();
    }
  file_read.close();
  }
}
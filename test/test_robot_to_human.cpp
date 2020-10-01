// gtest
#include <gtest/gtest.h>

//romea
#include <containers/Eigen/VectorOfEigenVector.hpp>
#include "rtls_localisation_position_estimator.hpp"
#include "rtls_localisation_simple_trilateration.hpp"



//-----------------------------------------------------------------------------
TEST(TestRobotToHuman, testPositionEstimator2A)
{
  Eigen::Vector3d leaderTagPosition(5,2,2);

  romea::VectorOfEigenVector3d followerTagPositions(2);
  followerTagPositions[0]=Eigen::Vector3d(0, 0.3,1);
  followerTagPositions[1]=Eigen::Vector3d(0,-0.3,1);

  romea::RTLSLocalisationRangeVector ranges(2);
  ranges[0]=(leaderTagPosition-followerTagPositions[0]).norm();
  ranges[1]=(leaderTagPosition-followerTagPositions[1]).norm();

  romea::RTLSLocalisationPositionEstimator estimator(0.001);
  estimator.loadGeometry(followerTagPositions);
  EXPECT_EQ(estimator.init(ranges),true);
  estimator.estimate(20,0.02);

  Eigen::Vector2d leaderTagEstimatedPosition = estimator.getEstimate();

  EXPECT_NEAR(leaderTagPosition.x(),leaderTagEstimatedPosition.x(),0.001);
  EXPECT_NEAR(leaderTagPosition.y(),leaderTagEstimatedPosition.y(),0.001);

}

//-----------------------------------------------------------------------------
TEST(TestRobotToHuman, testPositionEstimator3AUp)
{
  Eigen::Vector3d leaderTagPosition(-4,6,1);

  romea::VectorOfEigenVector3d followerTagPositions(3);
  followerTagPositions[0]= Eigen::Vector3d(0, 0.6,2);
  followerTagPositions[1]= Eigen::Vector3d(0,-0.6,1.5);
  followerTagPositions[2]= Eigen::Vector3d(1,0,1.8);

  romea::RTLSLocalisationRangeVector ranges(3);
  ranges[0]=(leaderTagPosition-followerTagPositions[0]).norm();
  ranges[1]=(leaderTagPosition-followerTagPositions[1]).norm();
  ranges[2]=(leaderTagPosition-followerTagPositions[2]).norm();

  romea::RTLSLocalisationPositionEstimator estimator(0.001);
  estimator.loadGeometry(followerTagPositions);
  EXPECT_EQ(estimator.init(ranges),true);
  estimator.estimate(20,0.02);

  Eigen::Vector2d leaderTagEstimatedPosition = estimator.getEstimate();

  EXPECT_NEAR(leaderTagPosition.x(),leaderTagEstimatedPosition.x(),0.001);
  EXPECT_NEAR(leaderTagPosition.y(),leaderTagEstimatedPosition.y(),0.001);

}

//-----------------------------------------------------------------------------
TEST(TestRobotToHuman, testPositionEstimator3ADown)
{
  Eigen::Vector3d leaderTagPosition(-6,-7,1);

  romea::VectorOfEigenVector3d followerTagPositions(3);
  followerTagPositions[0]= Eigen::Vector3d(0, 0.6,2);
  followerTagPositions[1]= Eigen::Vector3d(0,-0.6,1.5);
  followerTagPositions[2]= Eigen::Vector3d(-2,0,1.8);

  romea::RTLSLocalisationRangeVector ranges(3);
  ranges[0]=(leaderTagPosition-followerTagPositions[0]).norm();
  ranges[1]=(leaderTagPosition-followerTagPositions[1]).norm();
  ranges[2]=(leaderTagPosition-followerTagPositions[2]).norm();

  romea::RTLSLocalisationPositionEstimator estimator(0.001);
  estimator.loadGeometry(followerTagPositions);
  EXPECT_EQ(estimator.init(ranges),true);
  estimator.estimate(20,0.02);

  Eigen::Vector2d leaderTagEstimatedPosition = estimator.getEstimate();

  EXPECT_NEAR(leaderTagPosition.x(),leaderTagEstimatedPosition.x(),0.001);
  EXPECT_NEAR(leaderTagPosition.y(),leaderTagEstimatedPosition.y(),0.001);

}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

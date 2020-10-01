// gtest
#include <gtest/gtest.h>

//romea
#include <containers/Eigen/VectorOfEigenVector.hpp>
#include "rtls_localisation_pose_estimator.hpp"
#include "rtls_localisation_simple_trilateration.hpp"

//-----------------------------------------------------------------------------
TEST(TestRobotToRobot, testRTLS)
{

  std::vector<int> leaderRobotTagIds{0,1,2};
  std::vector<int> followerRobotTagIds{0,1,2};
  romea::VectorOfEigenVector3d leaderRobotTagPositions
  {Eigen::Vector3d(0,-0.21,0.39),Eigen::Vector3d(0, 0.21,0.39),Eigen::Vector3d(0.85, 0, 0.44)};
  romea::VectorOfEigenVector3d followerRobotTaPositions
  {Eigen::Vector3d(0,-0.3,1.01),Eigen::Vector3d(0, 0.3,1.01),Eigen::Vector3d(0.44,0,0.71)};

  romea::R2RRTLS r2rRtls(leaderRobotTagIds,
                         leaderRobotTagPositions,
                         followerRobotTagIds,
                         followerRobotTaPositions);

  romea::R2RPoseEstimator poseEstimator_(0.001);

  //  r2rRtls.addLeaderRobotTag(0,Eigen::Vector3d(0,-0.21,0.39));
  //  r2rRtls.addLeaderRobotTag(1,Eigen::Vector3d(0, 0.21,0.39));
  //  r2rRtls.addLeaderRobotTag(2,Eigen::Vector3d(0.85, 0, 0.44));
  //  r2rRtls.addFollowerRobotTag(0,Eigen::Vector3d(0,-0.3,1.01));
  //  r2rRtls.addFollowerRobotTag(1,Eigen::Vector3d(0, 0.3,1.01));
  //  r2rRtls.addFollowerRobotTag(2,Eigen::Vector3d(0.44,0,0.71));
  //  r2rRtls.setPollRate(20);

  double rho=5;
  double theta =0;
  double course=-M_PI;
  for(; theta<2*M_PI;theta+=M_PI/4, course+=M_PI/3)
  {

    double leaderX = rho*std::cos(theta);
    double leaderY = rho*std::sin(theta);
    double leaderCourse = romea::between0And2Pi(course);

    Eigen::Matrix3d R= romea::eulerAnglesToRotation3D(Eigen::Vector3d(0,0,leaderCourse));
    Eigen::Vector3d T(leaderX,leaderY,0);

    {

      std::vector<double> ranges(9);

      for( size_t i=0; i<3;i++)
      {
        for( size_t j=0; j<3; j++)
        {
          ranges[i*3+j] = ((R*r2rRtls.getPositionOfLeaderRobotTags()[i]+T)-
                           r2rRtls.getPositionOfFollowerRobotTags()[j]).norm();

        }
      }

      r2rRtls.setRange(0,0,romea::Range(romea::durationFromSecond(10.000),ranges[0],0.2,0,0,0,0));
      r2rRtls.setRange(1,0,romea::Range(romea::durationFromSecond(10.050),ranges[1],0.2,0,0,0,1));
      r2rRtls.setRange(2,0,romea::Range(romea::durationFromSecond(10.100),ranges[2],0.2,0,0,0,2));
      r2rRtls.setRange(0,1,romea::Range(romea::durationFromSecond(10.150),ranges[3],0.2,0,0,1,0));
      r2rRtls.setRange(1,1,romea::Range(romea::durationFromSecond(10.200),ranges[4],0.2,0,0,1,1));
      r2rRtls.setRange(2,1,romea::Range(romea::durationFromSecond(10.250),ranges[5],0.2,0,0,1,2));
      r2rRtls.setRange(0,2,romea::Range(romea::durationFromSecond(10.300),ranges[6],0.2,0,0,2,0));
      r2rRtls.setRange(1,2,romea::Range(romea::durationFromSecond(10.350),ranges[7],0.2,0,0,2,1));
      r2rRtls.setRange(2,2,romea::Range(romea::durationFromSecond(10.400),ranges[8],0.2,0,0,2,2));

      poseEstimator_.init(r2rRtls.getPositionOfLeaderRobotTags(),
                          r2rRtls.getPositionOfFollowerRobotTags(),
                          r2rRtls.getRanges());

      ASSERT_EQ(poseEstimator_.estimate(10,0.2), true);
      EXPECT_NEAR(leaderX,poseEstimator_.getEstimate()[0],0.01);
      EXPECT_NEAR(leaderY,poseEstimator_.getEstimate()[1],0.01);
      EXPECT_NEAR(leaderCourse,poseEstimator_.getEstimate()[2],0.01);

    }
  }
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

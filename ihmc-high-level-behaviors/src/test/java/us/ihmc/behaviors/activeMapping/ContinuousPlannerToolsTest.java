package us.ihmc.behaviors.activeMapping;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static org.junit.jupiter.api.Assertions.*;

public class ContinuousPlannerToolsTest
{
   // These could be put into tunable parameters but for now they were left here
   private static final float X_RANDOM_MARGIN = 0.2f;
   private static final float NOMINAL_STANCE_WIDTH = 0.22f;

   private final ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();

   private final FramePose3D walkingStartMidPose = new FramePose3D();
   private final SideDependentList<FramePose3D> stancePose = new SideDependentList<>();

   @BeforeEach
   public void setupVariables()
   {
      // Set the starting point to be half a meter forward in the X direction
      walkingStartMidPose.setX(0.5);
      walkingStartMidPose.setY(0.0);
      walkingStartMidPose.setZ(0.0);

      // Set the starting stance pose to set the goal forward from
      for (RobotSide side : RobotSide.values)
      {
         stancePose.set(side, new FramePose3D());
         stancePose.get(side).setX(0.5);
         stancePose.get(side).setY((side == RobotSide.LEFT ? 0.1 : -0.1));
         stancePose.get(side).setZ(0.0);
      }
   }

   /**
    * This test checks to make sure the goal poses are at the expected distance from the start, and at the expected height from the start, and that the width
    * is
    * about as expected (not super strict on the accuracy of the width). Meant to test
    * {@link ContinuousPlannerTools#setRandomizedStraightGoalPoses(FramePose3D, SideDependentList, float, float, float, float)}
    */
   @Test
   public void testRandomizedStraightGoalPoses()
   {
      // Pass everything to the tools method to get the forward goal position
      SideDependentList<FramePose3D> goalPoses = ContinuousPlannerTools.setRandomizedStraightGoalPoses(walkingStartMidPose,
                                                                                                       stancePose,
                                                                                                       (float) continuousHikingParameters.getGoalPoseForwardDistance(),
                                                                                                       X_RANDOM_MARGIN,
                                                                                                       (float) continuousHikingParameters.getGoalPoseUpDistance(),
                                                                                                       NOMINAL_STANCE_WIDTH);

      // These are the maximum and minimum values where the goal pose should be able to exist
      double maxForwardGoalDistance = walkingStartMidPose.getX() + continuousHikingParameters.getGoalPoseForwardDistance() + X_RANDOM_MARGIN;
      double minForwardGoalDistance = walkingStartMidPose.getX() + continuousHikingParameters.getGoalPoseForwardDistance() - X_RANDOM_MARGIN;

      for (RobotSide robotSide : RobotSide.values)
      {
         assertTrue(goalPoses.get(robotSide).getX() < maxForwardGoalDistance,
                    "The distance was ( " + goalPoses.get(robotSide).getX() + " ) and should be less then ( " + maxForwardGoalDistance + " )");
         assertTrue(goalPoses.get(robotSide).getX() > minForwardGoalDistance,
                    "The distance was ( " + goalPoses.get(robotSide).getX() + " ) and should be more then ( " + minForwardGoalDistance + " )");
      }

      // Need to get the stance pose z height
      FramePose3D stanceMidPose = new FramePose3D();
      stanceMidPose.interpolate(stancePose.get(RobotSide.LEFT), stancePose.get(RobotSide.RIGHT), 0.5);

      // This is the expected height where the goal pose should be
      double expectedGoalHeight = stanceMidPose.getZ() + continuousHikingParameters.getGoalPoseUpDistance() - walkingStartMidPose.getZ();

      for (RobotSide robotSide : RobotSide.values)
      {
         // Round to three decimal places and call it a day
         double roundedGoalPose = (double) Math.round(goalPoses.get(robotSide).getZ() * 1000) / 1000;

         assertEquals(roundedGoalPose,
                      expectedGoalHeight,
                      "The height was ( " + goalPoses.get(robotSide).getZ() + " ) and should equal ( " + expectedGoalHeight + " )");
      }

      double epsilon = 0.1; // Not too worried about the specifics of the width right now, just that is roughly as expected
      double expectedWidthOfGoalPoses = NOMINAL_STANCE_WIDTH + epsilon / 2;

      for (RobotSide robotSide : RobotSide.values)
      {
         // Need to take absolute value because one Y will be negative, this makes sure both feet are checked accurately
         assertTrue(Math.abs(goalPoses.get(robotSide).getY()) < expectedWidthOfGoalPoses,
                    "The width for each side should be less then ( " + expectedWidthOfGoalPoses + " )");
      }
   }

   /**
    * Checks the distance from the robot position to a specific goal pose. This test is pretty straight forward, it was made because it was quick to make and
    * might prove valuable in the future. Meant to test
    * {@link ContinuousPlannerTools#getDistanceFromRobotToGoalPoseOnXYPlane(Point3DReadOnly, SideDependentList)}
    */
   @Test
   public void testGetDistanceFromRobotToGoalPoseOnXYPlane()
   {
      SideDependentList<FramePose3D> goalPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         goalPoses.set(robotSide, new FramePose3D());
         goalPoses.get(robotSide).setX(0.5);
         goalPoses.get(robotSide).setY(0.0);
         goalPoses.get(robotSide).setZ(0.0);
      }

      Point3D robotLocation = new Point3D();
      robotLocation.setX(0.0);
      robotLocation.setY(0.0);
      robotLocation.setZ(0.0);

      // This is based on how the test is set up currently
      double expectedDistanceFromRobot = 0.5;

      double actualDistanceFromRobot = ContinuousPlannerTools.getDistanceFromRobotToGoalPoseOnXYPlane(robotLocation, goalPoses);

      assertEquals(expectedDistanceFromRobot, actualDistanceFromRobot);
   }
}

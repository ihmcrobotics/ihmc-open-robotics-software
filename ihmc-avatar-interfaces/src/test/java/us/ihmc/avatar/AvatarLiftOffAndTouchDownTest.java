package us.ihmc.avatar;

import java.util.Arrays;

import org.apache.commons.math3.util.Precision;

import controller_msgs.msg.dds.AdjustFootstepMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AvatarLiftOffAndTouchDownTest
{
   private static final double PITCH_EPSILON = Math.toRadians(3.0);

   public static boolean doStep(DRCRobotModel robotModel, DRCSimulationTestHelper testHelper, double stepLength, double startPitch, double finalPitch,
                                double footLength)
         throws SimulationExceededMaximumTimeException
   {
      return doStep(robotModel, testHelper, stepLength, startPitch, finalPitch, footLength, 0.0);
   }

   public static boolean doStep(DRCRobotModel robotModel, DRCSimulationTestHelper testHelper, double stepLength, double startPitch, double finalPitch,
                                double footLength, double adjustmentX)
         throws SimulationExceededMaximumTimeException
   {
      double swingDuration = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double swingHeight = Math.max(0.05, robotModel.getWalkingControllerParameters().getSteppingParameters().getDefaultSwingHeightFromStanceFoot());
      double touchdownVelocity = robotModel.getWalkingControllerParameters().getSwingTrajectoryParameters().getDesiredTouchdownVelocity();
      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage message = new FootstepDataListMessage();
      FootstepDataMessage step = message.getFootstepDataList().add();
      MovingReferenceFrame soleFrame = testHelper.getReferenceFrames().getSoleFrame(side);
      FramePose3D footstepPose = new FramePose3D(soleFrame);
      footstepPose.setX(stepLength);

      footstepPose.changeFrame(ReferenceFrame.getWorldFrame());
      footstepPose.setZ(0.0);
      footstepPose.setOrientationYawPitchRoll(footstepPose.getYaw(), 0.0, 0.0);
      step.setRobotSide(side.toByte());
      step.getLocation().set(footstepPose.getPosition());
      step.getOrientation().set(footstepPose.getOrientation());

      step.setTrajectoryType(TrajectoryType.WAYPOINTS.toByte());
      step.setSwingDuration(swingDuration);
      step.setSwingTrajectoryBlendDuration(0.5 * swingDuration);
      double partialFootholdDuration = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime() / 2.0;
      step.setLiftoffDuration(partialFootholdDuration);
      step.setTouchdownDuration(partialFootholdDuration);

      SE3TrajectoryPointMessage waypoint0 = step.getSwingTrajectory().add();
      waypoint0.setTime(0.0);
      FramePose3D waypointPose0 = new FramePose3D(soleFrame);
      waypointPose0.setOrientationYawPitchRoll(waypointPose0.getYaw(), startPitch, waypointPose0.getRoll());
      waypointPose0.setZ(waypointPose0.getZ() + footLength * Math.sin(Math.abs(startPitch)) / 2.0);
      waypointPose0.changeFrame(ReferenceFrame.getWorldFrame());

      SE3TrajectoryPointMessage waypoint1 = step.getSwingTrajectory().add();
      waypoint1.setTime(swingDuration / 2.0);

      SE3TrajectoryPointMessage waypoint2 = step.getSwingTrajectory().add();
      waypoint2.setTime(swingDuration);
      FramePose3D waypointPose2 = new FramePose3D(footstepPose);
      waypointPose2.changeFrame(soleFrame);
      waypointPose2.setOrientationYawPitchRoll(waypointPose2.getYaw(), finalPitch, waypointPose2.getRoll());
      waypointPose2.changeFrame(ReferenceFrame.getWorldFrame());
      waypointPose2.setZ(waypointPose2.getZ() + footLength * Math.sin(Math.abs(finalPitch)) / 2.0);

      FramePose3D waypointPose1 = new FramePose3D(waypointPose0);
      waypointPose1.interpolate(waypointPose2, 0.5);
      waypointPose1.setZ(swingHeight);

      PositionOptimizedTrajectoryGenerator optimizer = new PositionOptimizedTrajectoryGenerator(-1, 1);
      optimizer.setEndpointConditions(waypointPose0.getPosition(), new FrameVector3D(), waypointPose2.getPosition(), new FrameVector3D());
      optimizer.setWaypoints(Arrays.asList(waypointPose1.getPosition()));
      optimizer.initialize();
      FrameVector3D waypoint1LinearVelocity = new FrameVector3D();
      optimizer.getWaypointVelocity(0, waypoint1LinearVelocity);
      waypoint1LinearVelocity.scale(1.0 / swingDuration);
      FrameVector3D waypoint1AngularVelocity = new FrameVector3D(soleFrame);
      waypointPose0.changeFrame(soleFrame);
      waypointPose2.changeFrame(soleFrame);
      waypoint1AngularVelocity.setY((waypointPose2.getPitch() - waypointPose0.getPitch()) / swingDuration);
      waypoint1AngularVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      waypointPose0.changeFrame(ReferenceFrame.getWorldFrame());
      waypointPose2.changeFrame(ReferenceFrame.getWorldFrame());

      waypoint0.getPosition().set(waypointPose0.getPosition());
      waypoint0.getOrientation().set(waypointPose0.getOrientation());
      waypoint1.getPosition().set(waypointPose1.getPosition());
      waypoint1.getLinearVelocity().set(waypoint1LinearVelocity);
      waypoint1.getOrientation().set(waypointPose1.getOrientation());
      waypoint1.getAngularVelocity().set(waypoint1AngularVelocity);
      waypoint2.getPosition().set(waypointPose2.getPosition());
      waypoint2.getLinearVelocity().set(0.0, 0.0, touchdownVelocity);
      waypoint2.getOrientation().set(waypointPose2.getOrientation());
      waypoint2.getAngularVelocity().set(waypoint1AngularVelocity);

      boolean success = true;
      double timeOffset = 0.1;

      success &= checkFullContact(testHelper, side, robotModel);
      success &= checkFootPitch(testHelper, 0.0, side);

      testHelper.publishToController(message);
      testHelper.simulateAndBlockAndCatchExceptions(robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime() - timeOffset);
      if (!MathTools.epsilonEquals(startPitch, 0.0, Math.toRadians(5.0)))
         success &= checkPartialContact(testHelper, side, robotModel);
      testHelper.simulateAndBlockAndCatchExceptions(timeOffset);
      success &= checkFootPitch(testHelper, startPitch, side);

      testHelper.simulateAndBlockAndCatchExceptions(robotModel.getWalkingControllerParameters().getDefaultSwingTime() / 3.0);

      if (!Precision.equals(adjustmentX, 0.0))
      {
         AdjustFootstepMessage adjustFootstepMessage = new AdjustFootstepMessage();
         adjustFootstepMessage.getLocation().set(step.getLocation());
         adjustFootstepMessage.getOrientation().set(step.getOrientation());
         adjustFootstepMessage.setRobotSide(side.toByte());
         adjustFootstepMessage.getLocation().addX(adjustmentX);
         testHelper.publishToController(adjustFootstepMessage);
      }

      testHelper.simulateAndBlockAndCatchExceptions(robotModel.getWalkingControllerParameters().getDefaultSwingTime() * 2.0 / 3.0);
      success &= checkFootPitch(testHelper, finalPitch, side);
      testHelper.simulateAndBlockAndCatchExceptions(timeOffset);
      if (!MathTools.epsilonEquals(finalPitch, 0.0, Math.toRadians(5.0)))
         success &= checkPartialContact(testHelper, side, robotModel);
      testHelper.simulateAndBlockAndCatchExceptions(Math.max(robotModel.getWalkingControllerParameters().getDefaultFinalTransferTime(), partialFootholdDuration)
            - timeOffset);

      success &= checkFootPitch(testHelper, 0.0, side);
      testHelper.simulateAndBlockAndCatchExceptions(timeOffset);
      success &= checkFullContact(testHelper, side, robotModel);
      testHelper.simulateAndBlockAndCatchExceptions(0.25 - timeOffset);

      return success;
   }

   private static boolean checkPartialContact(DRCSimulationTestHelper testHelper, RobotSide side, DRCRobotModel robotModel)
   {
      int contactPoints = robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings().getNumberOfContactPointsPerContactableBody();
      String prefix = testHelper.getReferenceFrames().getSoleFrame(side).getName();
      int inContact = 0;
      for (int i = 0; i < contactPoints; i++)
      {
         if (((YoBoolean) testHelper.getYoVariable(prefix + "InContact" + i)).getValue())
            inContact++;
      }
      boolean atLeastOneContact = inContact > 0;
      boolean notFullContact = inContact < contactPoints;
      if (!atLeastOneContact)
         System.out.println("At time " + testHelper.getRobot().getTime() + ": Foot was not in contact at all but expected partial contact.");
      if (!notFullContact)
         System.out.println("At time " + testHelper.getRobot().getTime() + ": Foot was in full contact but expected partial contact.");
      return atLeastOneContact && notFullContact;
   }

   private static boolean checkFullContact(DRCSimulationTestHelper testHelper, RobotSide side, DRCRobotModel robotModel)
   {
      int contactPoints = robotModel.getWalkingControllerParameters().getMomentumOptimizationSettings().getNumberOfContactPointsPerContactableBody();
      String prefix = testHelper.getReferenceFrames().getSoleFrame(side).getName();
      int inContact = 0;
      for (int i = 0; i < contactPoints; i++)
      {
         if (((YoBoolean) testHelper.getYoVariable(prefix + "InContact" + i)).getValue())
            inContact++;
      }
      boolean fullContact = inContact == contactPoints;
      if (!fullContact)
         System.out.println("At time " + testHelper.getRobot().getTime() + ": Foot was not in full contact.");
      return fullContact;
   }

   private static boolean checkFootPitch(DRCSimulationTestHelper testHelper, double expectedPitch, RobotSide side)
   {
      MovingReferenceFrame soleFrame = testHelper.getReferenceFrames().getSoleFrame(side);
      MovingReferenceFrame soleZUpFrame = testHelper.getReferenceFrames().getSoleZUpFrame(side);
      FrameQuaternion soleOrientation = new FrameQuaternion(soleFrame);
      soleOrientation.changeFrame(soleZUpFrame);
      double actualPitch = soleOrientation.getPitch();

      String footName = testHelper.getControllerFullRobotModel().getFoot(side).getName();
      FrameQuaternion desiredSoleOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame());
      desiredSoleOrientation.set(EndToEndTestTools.findFeedbackControllerDesiredOrientation(footName, testHelper.getSimulationConstructionSet()));
      desiredSoleOrientation.changeFrame(soleZUpFrame);
      double actualDesiredPitch = desiredSoleOrientation.getPitch();

      boolean actualPitchEquals = MathTools.epsilonEquals(expectedPitch, actualPitch, PITCH_EPSILON);
      boolean desiredPitchEquals = MathTools.epsilonEquals(expectedPitch, actualDesiredPitch, PITCH_EPSILON);

      if (!actualPitchEquals)
         System.out.println("At time " + testHelper.getRobot().getTime() + ": actual pitch " + actualPitch + " was not close enough to expected "
               + expectedPitch + ".");
      if (!desiredPitchEquals)
         System.out.println("At time " + testHelper.getRobot().getTime() + ": desired pitch " + actualDesiredPitch + " was not close enough to expected "
               + expectedPitch + ".");

      return actualPitchEquals && desiredPitchEquals;
   }

   public static DRCSimulationTestHelper setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException
   {
      return setupTest(robotModel, 0.0);
   }

   public static DRCSimulationTestHelper setupTest(DRCRobotModel robotModel, double initialYaw) throws SimulationExceededMaximumTimeException
   {
      SimulationTestingParameters simulationTestParameters = SimulationTestingParameters.createFromSystemProperties();
      DRCSimulationTestHelper testHelper = new DRCSimulationTestHelper(simulationTestParameters, robotModel, new FlatGroundEnvironment());
      testHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(initialYaw));
      testHelper.createSimulation("LiftOffAndTouchDownTest");
      testHelper.setupCameraForUnitTest(new Point3D(0.3, 0.0, 0.3), new Point3D(1.0, 4.0, 1.0));
      testHelper.simulateAndBlockAndCatchExceptions(0.25);
      return testHelper;
   }
}

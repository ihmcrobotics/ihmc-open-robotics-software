package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;

public abstract class AvatarFootstepDataMessageSwingTrajectoryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   public void testSwingTrajectoryInWorld() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      ThreadTools.sleep(1000);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(8.0, -8.0, 5.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(1.5, 0.0, 0.8);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      
      double swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime() * 3.0;
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage(swingTime, transferTime);
      
      // step in place but do some fancy foot motion
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint footPosition = new FramePoint(soleFrame);
      FrameOrientation footOrientation = new FrameOrientation(soleFrame);
      
      FootstepDataMessage footstep = new FootstepDataMessage();
      footstep.setRobotSide(robotSide);
      footstep.setTrajectoryType(TrajectoryType.WAYPOINTS);
      footstep.setTrajectoryReferenceFrameId(worldFrame);
      footstep.setTimings(swingTime, transferTime);
      
      double radius = 0.075;
      FramePoint circleCenter = new FramePoint(soleFrame, 0.0, 0.0, 0.15);
      int points = 8;
      
      SE3TrajectoryPointMessage[] waypoints = new SE3TrajectoryPointMessage[points];
      for (int i = 0; i < points; i++)
      {
         double percentInCircle = i / (points - 1.0);
         double percentInSwing = (i + 1.0) / (points + 1.0);
         double angleInCircle = 2.0 * Math.PI * percentInCircle;
         
         double xOffset = Math.sin(angleInCircle) * radius;
         double zOffset = -Math.cos(angleInCircle) * radius;
         FramePoint waypointPosition = new FramePoint(circleCenter);
         waypointPosition.add(xOffset, 0.0, zOffset);
         waypointPosition.changeFrame(worldFrame);
         
         FrameOrientation waypointOrientation = new FrameOrientation(soleFrame);
         waypointOrientation.changeFrame(worldFrame);
         
         SE3TrajectoryPointMessage waypoint = new SE3TrajectoryPointMessage();
         waypoint.setTime(percentInSwing * swingTime);
         waypoint.setPosition(waypointPosition.getPoint());
         waypoint.setOrientation(waypointOrientation.getQuaternion());
         
         waypoints[i] = waypoint;
      }

      footPosition.changeFrame(worldFrame);
      footOrientation.changeFrame(worldFrame);
      footstep.setLocation(footPosition.getPoint());
      footstep.setOrientation(footOrientation.getQuaternion());
      footstep.setSwingTrajectory(waypoints);
      
      footstepDataList.add(footstep);
      drcSimulationTestHelper.send(footstepDataList);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(swingTime + 2.0 * transferTime));
   }
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters = null;
   }
}

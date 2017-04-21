package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
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
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, className, startingLocation, simulationTestingParameters, robotModel);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(0.0, -3.0, 1.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(0.0, 0.0, 0.2);
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      
      double swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime() * 3.0;
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double initialTransferTime = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage(swingTime, transferTime);
      footstepDataList.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS);
      
      // step in place but do some fancy foot motion
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint footPosition = new FramePoint(soleFrame);
      FrameOrientation footOrientation = new FrameOrientation(soleFrame);
      
      FootstepDataMessage footstep = new FootstepDataMessage();
      footstep.setRobotSide(robotSide);
      footstep.setTrajectoryType(TrajectoryType.WAYPOINTS);
      footstep.setTrajectoryReferenceFrameId(worldFrame);
      footstep.setTimings(swingTime, initialTransferTime);

      double radius = 0.075;
      FramePoint circleCenter = new FramePoint(soleFrame, 0.0, 0.0, 0.1);
      int points = 8;
      double timeBetweenWaypoints = swingTime / (points + 1);
      
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
         
         FrameVector waypointLinearVelocity = new FrameVector(soleFrame);
         if (i > 0 && i < points - 1)
         {
            double scale = 1.0 / timeBetweenWaypoints;
            double xVelocity = scale * Math.cos(angleInCircle) * radius;
            double zVelocity = scale * Math.sin(angleInCircle) * radius;
            waypointLinearVelocity.set(xVelocity, 0.0, zVelocity);
         }
         waypointLinearVelocity.changeFrame(worldFrame);
         
         FrameOrientation waypointOrientation = new FrameOrientation(soleFrame);
         waypointOrientation.changeFrame(worldFrame);
         
         SE3TrajectoryPointMessage waypoint = new SE3TrajectoryPointMessage();
         waypoint.setTime(percentInSwing * swingTime);
         waypoint.setPosition(waypointPosition.getPoint());
         waypoint.setLinearVelocity(waypointLinearVelocity.getVector());
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
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(initialTransferTime));
      String format = EuclidCoreIOTools.getStringFormat(6, 4);

      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String swingStateNamespace = sidePrefix + FootControlModule.class.getSimpleName();

      String namePrefix = sidePrefix + "Foot";
      String typeName = namePrefix + TrajectoryType.class.getSimpleName();
      String posName = namePrefix + "DesiredSolePositionInWorld";
      String oriName = namePrefix + "DesiredSoleOrientationInWorld";
      String linName = namePrefix + "DesiredSoleLinearVelocityInWorld";
      String angName = namePrefix + "DesiredSoleAngularVelocityInWorld";
      
      double poseEpsilon = 0.01;
      double velocityEpcilon = 0.1;
      
      for (int i = 0; i < points; i ++) 
      {
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeBetweenWaypoints));
         SE3TrajectoryPointMessage expectedDesired = waypoints[i];

         @SuppressWarnings("unchecked")
         EnumYoVariable<TrajectoryType> currentTrajectoryType = (EnumYoVariable<TrajectoryType>) scs.getVariable(swingStateNamespace, typeName);
         Point3D currentDesiredPosition = EndToEndHandTrajectoryMessageTest.findPoint3d(swingStateNamespace, posName, scs);
         Quaternion currentDesiredOrientation = EndToEndHandTrajectoryMessageTest.findQuat4d(swingStateNamespace, oriName, scs);
         Vector3D currentDesiredLinearVelocity = EndToEndHandTrajectoryMessageTest.findVector3d(swingStateNamespace, linName, scs);
         Vector3D currentDesiredAngularVelocity = EndToEndHandTrajectoryMessageTest.findVector3d(swingStateNamespace, angName, scs);

         assertEquals("Unexpected Trajectory Type", TrajectoryType.WAYPOINTS, currentTrajectoryType.getEnumValue());
         EuclidCoreTestTools.assertTuple3DEquals("Position", expectedDesired.position, currentDesiredPosition, poseEpsilon, format);
         EuclidCoreTestTools.assertQuaternionEqualsSmart("Orientation", expectedDesired.orientation, currentDesiredOrientation, poseEpsilon, format);
         EuclidCoreTestTools.assertTuple3DEquals("LinearVelocity", expectedDesired.linearVelocity, currentDesiredLinearVelocity, velocityEpcilon, format);
         EuclidCoreTestTools.assertTuple3DEquals("AngularVelocity", expectedDesired.angularVelocity, currentDesiredAngularVelocity, velocityEpcilon, format);
      }
      
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeBetweenWaypoints + transferTime));
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

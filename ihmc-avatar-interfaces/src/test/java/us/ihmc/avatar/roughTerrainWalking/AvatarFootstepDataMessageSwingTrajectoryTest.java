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
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFrameVariableNameTools;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.commons.thread.ThreadTools;

public abstract class AvatarFootstepDataMessageSwingTrajectoryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   /**
    * Method used to scale down trajectories for different robots.
    * @return shinLength + thighLength of the robot
    */
   public abstract double getLegLength();

   public void testSwingTrajectoryInWorld() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.createSimulation(className);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(0.0, -3.0, 1.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(0.0, 0.0, 0.2);
      ThreadTools.sleep(1000);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      double robotScale = getLegLength();
      double pelvisTrajectoryTime = 0.5;
      PelvisHeightTrajectoryMessage pelvisHeightMessage = new PelvisHeightTrajectoryMessage(pelvisTrajectoryTime, 1.1 * robotScale);
      drcSimulationTestHelper.send(pelvisHeightMessage);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(pelvisTrajectoryTime));

      double swingTime = 2.0;
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double initialTransferTime = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage(swingTime, transferTime);
      footstepDataList.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS);

      // step in place but do some fancy foot motion
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint3D footPosition = new FramePoint3D(soleFrame);
      FrameQuaternion footOrientation = new FrameQuaternion(soleFrame);

      FootstepDataMessage footstep = new FootstepDataMessage();
      footstep.setRobotSide(robotSide);
      footstep.setTrajectoryType(TrajectoryType.WAYPOINTS);
      footstep.setTimings(swingTime, initialTransferTime);

      double radius = robotScale * 0.10;
      double pitch = Math.toRadians(10.0);
      FramePoint3D circleCenter = new FramePoint3D(soleFrame, 0.0, 0.0, robotScale * 0.15);
      int points = 6;

      SE3TrajectoryPointMessage[] waypoints = new SE3TrajectoryPointMessage[points];
      for (int i = 0; i < points; i++)
      {
         double percentInCircle = i / (points - 1.0);
         double percentInSwing = (i + 1.0) / (points + 1.0);
         double angleInCircle = 2.0 * Math.PI * percentInCircle;

         double xOffset = Math.sin(angleInCircle) * radius;
         double zOffset = -Math.cos(angleInCircle) * radius;
         FramePoint3D waypointPosition = new FramePoint3D(circleCenter);
         waypointPosition.add(xOffset, 0.0, zOffset);
         waypointPosition.changeFrame(worldFrame);

         FrameVector3D waypointLinearVelocity = new FrameVector3D(soleFrame);
         if (i > 0 && i < points - 1)
         {
            double timeBetweenWaypoints = swingTime / (points + 1);
            double scale = 1.0 / timeBetweenWaypoints;
            double xVelocity = scale * Math.cos(angleInCircle) * radius;
            double zVelocity = scale * Math.sin(angleInCircle) * radius;
            waypointLinearVelocity.set(xVelocity, 0.0, zVelocity);
         }
         waypointLinearVelocity.changeFrame(worldFrame);

         FrameQuaternion waypointOrientation = new FrameQuaternion(soleFrame);
         if (i % 2 == 0)
         {
            waypointOrientation.setYawPitchRoll(0.0, pitch, 0.0);
         }
         else
         {
            waypointOrientation.setYawPitchRoll(0.0, -pitch, 0.0);
         }
         waypointOrientation.changeFrame(worldFrame);

         SE3TrajectoryPointMessage waypoint = new SE3TrajectoryPointMessage();
         waypoint.setTime(percentInSwing * swingTime);
         waypoint.setPosition(waypointPosition);
         waypoint.setLinearVelocity(waypointLinearVelocity);
         waypoint.setOrientation(waypointOrientation);

         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(waypointPosition);
         sphere.addSphere(0.01, new YoAppearanceRGBColor(FootstepListVisualizer.defaultFeetColors.get(robotSide), 0.0));
         scs.addStaticLinkGraphics(sphere);

         waypoints[i] = waypoint;
      }

      footPosition.changeFrame(worldFrame);
      footOrientation.changeFrame(worldFrame);
      footstep.setLocation(footPosition);
      footstep.setOrientation(footOrientation);
      footstep.setSwingTrajectory(waypoints);

      footstepDataList.add(footstep);
      drcSimulationTestHelper.send(footstepDataList);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(initialTransferTime + getRobotModel().getControllerDT() * 4.0));

      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String format = EuclidCoreIOTools.getStringFormat(6, 4);
      String prefix = sidePrefix + "FootSwing";
      String linearNamespace = prefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String angularNamespace = prefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      for (int i = 0; i < waypoints.length; i++)
      {
         SE3TrajectoryPointMessage waypoint = waypoints[i];

         String suffix = "AtWaypoint" + (i + 1);

         String positionPrefix = YoFrameVariableNameTools.createName(prefix, "position", "");
         Tuple3DBasics desiredPosition = EndToEndHandTrajectoryMessageTest.findTuple3d(linearNamespace, positionPrefix, suffix, scs);
         String linearVelocityPrefix = YoFrameVariableNameTools.createName(prefix, "linearVelocity", "");
         Tuple3DBasics desiredLinearVelocity = EndToEndHandTrajectoryMessageTest.findTuple3d(linearNamespace, linearVelocityPrefix, suffix, scs);

         EuclidCoreTestTools.assertTuple3DEquals("Position", waypoint.position, desiredPosition, 1.0E-10, format);
         EuclidCoreTestTools.assertTuple3DEquals("Linear Velocity", waypoint.linearVelocity, desiredLinearVelocity, 1.0E-10, format);

         String orientationPrefix = YoFrameVariableNameTools.createName(prefix, "orientation", "");
         Quaternion desiredOrientation = EndToEndHandTrajectoryMessageTest.findQuat4d(angularNamespace, orientationPrefix, suffix, scs);
         String angularVelocityPrefix = YoFrameVariableNameTools.createName(prefix, "angularVelocity", "");
         Tuple3DBasics desiredAngularVelocity = EndToEndHandTrajectoryMessageTest.findTuple3d(angularNamespace, angularVelocityPrefix, suffix, scs);

         EuclidCoreTestTools.assertTuple4DEquals("Orientation", waypoint.orientation, desiredOrientation, 1.0E-10, format);
         EuclidCoreTestTools.assertTuple3DEquals("Angular Velocity", waypoint.angularVelocity, desiredAngularVelocity, 1.0E-10, format);
      }

      String currentIndexName = prefix + "CurrentWaypointIndex";
      String swingStateNamespace = sidePrefix + FootControlModule.class.getSimpleName();
      String typeName = sidePrefix + "Foot" + TrajectoryType.class.getSimpleName();

      @SuppressWarnings("unchecked")
      YoEnum<TrajectoryType> currentTrajectoryType = (YoEnum<TrajectoryType>) scs.getVariable(swingStateNamespace, typeName);
      YoVariable<?> currentWaypointIndex = scs.getVariable(linearNamespace, currentIndexName);

      assertEquals("Unexpected Trajectory Type", TrajectoryType.WAYPOINTS, currentTrajectoryType.getEnumValue());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(swingTime + transferTime));
      assertEquals("Swing Trajectory did not execute.", points, currentWaypointIndex.getValueAsLongBits());
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

package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.SwingTrajectoryCalculator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class AvatarFootstepDataMessageSwingTrajectoryTest implements MultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private PushRobotControllerSCS2 pushController;

   private Boolean pushAndAdjust;

   public void setPushAndAdjust(boolean pushAndAdjust)
   {
      this.pushAndAdjust = pushAndAdjust;
   }

   /**
    * Method used to scale down trajectories for different robots.
    * 
    * @return shinLength + thighLength of the robot
    */
   public abstract double getLegLength();

   @Test
   public void testSwingTrajectoryTouchdownSpeed()
   {
      runTestTouchdownSpeed();
   }

   @Test
   public void testSwingTrajectoryTouchdownWithAdjustment()
   {
      runTestTouchdownSpeed();
   }

   private void runTestTouchdownSpeed()
   {
      DRCRobotModel robotModel = setup(DRCObstacleCourseStartingLocation.DEFAULT);

      // Step forward with a late touchdown.
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint3D footPosition = new FramePoint3D(soleFrame);
      FrameQuaternion footOrientation = new FrameQuaternion(soleFrame);
      footPosition.changeFrame(worldFrame);
      footOrientation.changeFrame(worldFrame);

      double stepHeight = 0.1 * getLegLength();
      FramePoint3D touchdownPosition = new FramePoint3D(soleFrame, 0.4 * getLegLength(), 0.0, stepHeight);
      touchdownPosition.changeFrame(worldFrame);

      double swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double initialTransferTime = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());
      message.setAreFootstepsAdjustable(pushAndAdjust);

      ((YoBoolean) simulationTestHelper.findVariable("controllerSwingSpeedUpEnabled")).set(false);
      ((YoBoolean) simulationTestHelper.findVariable("leftFootSwingIsSpeedUpEnabled")).set(false);
      ((YoBoolean) simulationTestHelper.findVariable("rightFootSwingIsSpeedUpEnabled")).set(false);

      FootstepDataMessage footstep = message.getFootstepDataList().add();
      footstep.setRobotSide(robotSide.toByte());
      footstep.getLocation().set(touchdownPosition);
      footstep.getOrientation().set(footOrientation);

      double touchdownVelocity = -0.2;
      double lastPortion = 0.2;

      footstep.setTrajectoryType(TrajectoryType.WAYPOINTS.toByte());
      SE3TrajectoryPointMessage waypointA = footstep.getSwingTrajectory().add();
      waypointA.setTime(0.2 * swingTime);
      waypointA.getPosition().set(footPosition);
      waypointA.getPosition().addZ(stepHeight + 0.05);
      waypointA.getOrientation().set(footOrientation);
      waypointA.getLinearVelocity().setToZero();
      waypointA.getAngularVelocity().setToZero();

      SE3TrajectoryPointMessage waypointB = footstep.getSwingTrajectory().add();
      waypointB.setTime(2.0 * swingTime / 3.0);
      waypointB.getPosition().set(touchdownPosition);
      waypointB.getPosition().addZ(-lastPortion * swingTime * touchdownVelocity + 0.03);
      waypointB.getOrientation().set(footOrientation);
      waypointB.getLinearVelocity().setToZero();
      waypointB.getAngularVelocity().setToZero();

      SE3TrajectoryPointMessage waypointC = footstep.getSwingTrajectory().add();
      waypointC.setTime((1.0 - lastPortion) * swingTime);
      waypointC.getPosition().set(touchdownPosition);
      waypointC.getPosition().addZ(-lastPortion * swingTime * touchdownVelocity);
      waypointC.getOrientation().set(footOrientation);
      waypointC.getLinearVelocity().set(0.0, 0.0, touchdownVelocity);
      waypointC.getAngularVelocity().setToZero();

      SE3TrajectoryPointMessage touchdown = footstep.getSwingTrajectory().add();
      touchdown.setTime(swingTime);
      touchdown.getPosition().set(touchdownPosition);
      touchdown.getOrientation().set(footOrientation);
      touchdown.getLinearVelocity().set(0.0, 0.0, touchdownVelocity);
      touchdown.getAngularVelocity().setToZero();

      YoVariable desiredVelocity = simulationTestHelper.findVariable("leftFootControlModule", "leftFootSwingDesiredSoleLinearVelocityInWorldZ");

      // Push the robot to trigger footstep adjustment.
      if (pushAndAdjust)
      {
         FrameVector3D direction = new FrameVector3D(soleFrame, 1.0, 0.0, 0.0);
         direction.changeFrame(worldFrame);
         pushController.applyForceDelayed(time -> true, initialTransferTime + swingTime / 2.0, direction, 200.0, 0.1);
      }

      simulationTestHelper.publishToController(message);
      assertTrue(simulationTestHelper.simulateNow(initialTransferTime + (1.0 - lastPortion / 2.0) * swingTime));
      Assert.assertEquals(touchdownVelocity, desiredVelocity.getValueAsDouble(), 1.0e-10);
      assertTrue(simulationTestHelper.simulateNow(lastPortion));
      Assert.assertEquals(touchdownVelocity, desiredVelocity.getValueAsDouble(), 1.0e-10);
   }

   @Test
   public void testSwingTrajectoryInWorld()
   {
      DRCRobotModel robotModel = setup(DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI);

      double robotScale = getLegLength();
      double pelvisTrajectoryTime = 0.5;
      PelvisHeightTrajectoryMessage pelvisHeightMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(pelvisTrajectoryTime, 1.1 * robotScale);
      simulationTestHelper.publishToController(pelvisHeightMessage);
      assertTrue(simulationTestHelper.simulateNow(pelvisTrajectoryTime));

      double swingTime = 2.0;
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double initialTransferTime = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      FootstepDataListMessage footstepDataList = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      footstepDataList.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());

      // step in place but do some fancy foot motion
      RobotSide robotSide = RobotSide.LEFT;
      ReferenceFrame soleFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
      FramePoint3D footPosition = new FramePoint3D(soleFrame);
      FrameQuaternion footOrientation = new FrameQuaternion(soleFrame);

      FootstepDataMessage footstep = new FootstepDataMessage();
      footstep.setRobotSide(robotSide.toByte());
      footstep.setTrajectoryType(TrajectoryType.WAYPOINTS.toByte());
      footstep.setSwingDuration(swingTime);
      footstep.setTransferDuration(initialTransferTime);

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
         waypoint.getPosition().set(waypointPosition);
         waypoint.getLinearVelocity().set(waypointLinearVelocity);
         waypoint.getOrientation().set(waypointOrientation);

         VisualDefinitionFactory visualFactory = new VisualDefinitionFactory();
         visualFactory.appendTranslation(waypointPosition);
         visualFactory.addSphere(0.01, ColorDefinitions.rgb(FootstepListVisualizer.defaultFeetColors.get(robotSide).getRGB()));
         simulationTestHelper.addStaticVisuals(visualFactory.getVisualDefinitions());

         waypoints[i] = waypoint;
      }

      footPosition.changeFrame(worldFrame);
      footOrientation.changeFrame(worldFrame);
      footstep.getLocation().set(footPosition);
      footstep.getOrientation().set(footOrientation);
      MessageTools.copyData(waypoints, footstep.getSwingTrajectory());

      footstepDataList.getFootstepDataList().add().set(footstep);
      simulationTestHelper.publishToController(footstepDataList);
      assertTrue(simulationTestHelper.simulateNow(initialTransferTime + getRobotModel().getControllerDT() * 4.0));

      String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
      String format = EuclidCoreIOTools.getStringFormat(6, 4);
      String prefix = sidePrefix + "FootSwing";
      String linearNamespace = prefix + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName();
      String angularNamespace = prefix + MultipleWaypointsOrientationTrajectoryGenerator.class.getSimpleName();

      for (int i = 0; i < waypoints.length; i++)
      {
         SE3TrajectoryPointMessage waypoint = waypoints[i];

         String suffix = "AtWaypoint" + (i + 1);

         String positionPrefix = YoGeometryNameTools.assembleName(prefix, "position", "");
         Point3D desiredPosition = EndToEndTestTools.findPoint3D(linearNamespace, positionPrefix, suffix, simulationTestHelper);
         String linearVelocityPrefix = YoGeometryNameTools.assembleName(prefix, "linearVelocity", "");
         Vector3D desiredLinearVelocity = EndToEndTestTools.findVector3D(linearNamespace, linearVelocityPrefix, suffix, simulationTestHelper);

         EuclidCoreTestTools.assertEquals("Position", waypoint.getPosition(), desiredPosition, 1.0E-10, format);
         EuclidCoreTestTools.assertEquals("Linear Velocity", waypoint.getLinearVelocity(), desiredLinearVelocity, 1.0E-10, format);

         String orientationPrefix = YoGeometryNameTools.assembleName(prefix, "orientation", "");
         Quaternion desiredOrientation = EndToEndTestTools.findQuaternion(angularNamespace, orientationPrefix, suffix, simulationTestHelper);
         String angularVelocityPrefix = YoGeometryNameTools.assembleName(prefix, "angularVelocity", "");
         Vector3D desiredAngularVelocity = EndToEndTestTools.findVector3D(angularNamespace, angularVelocityPrefix, suffix, simulationTestHelper);

         EuclidCoreTestTools.assertEquals("Orientation", waypoint.getOrientation(), desiredOrientation, 1.0E-10, format);
         EuclidCoreTestTools.assertEquals("Angular Velocity", waypoint.getAngularVelocity(), desiredAngularVelocity, 1.0E-10, format);
      }

      String currentIndexName = prefix + "CurrentWaypointIndex";
      String swingStateNamespace = sidePrefix + "FootSwing" + SwingTrajectoryCalculator.class.getSimpleName();
      String typeName = sidePrefix + "FootSwing" + TrajectoryType.class.getSimpleName();

      @SuppressWarnings("unchecked")
      YoEnum<TrajectoryType> currentTrajectoryType = (YoEnum<TrajectoryType>) simulationTestHelper.findVariable(swingStateNamespace, typeName);
      YoVariable currentWaypointIndex = simulationTestHelper.findVariable(linearNamespace, currentIndexName);

      assertEquals("Unexpected Trajectory Type", TrajectoryType.WAYPOINTS, currentTrajectoryType.getEnumValue());
      assertTrue(simulationTestHelper.simulateNow(swingTime + transferTime));
      assertEquals("Swing Trajectory did not execute.", points, currentWaypointIndex.getValueAsLongBits());
   }

   private DRCRobotModel setup(DRCStartingLocation startingLocation)
   {
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(startingLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      simulationTestHelper.setCameraPosition(0.0, -3.0, 1.0);
      simulationTestHelper.setCameraFocusPosition(0.0, 0.0, 0.2);
      pushController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                   simulationTestHelper.getRobot(),
                                                   robotModel.createFullRobotModel().getChest().getParentJoint().getName(),
                                                   new Vector3D(0, 0, 0.15));
      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateNow(0.5));
      return robotModel;
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      pushAndAdjust = null;
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters = null;
      pushAndAdjust = null;
   }

}

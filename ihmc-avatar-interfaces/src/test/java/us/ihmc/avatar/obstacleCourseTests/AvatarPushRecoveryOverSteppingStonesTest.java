package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarPushRecoveryOverSteppingStonesTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private PushRobotControllerSCS2 pushRobotController;

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private double swingTime, totalMass;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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

      if (pushRobotController != null)
      {
         pushRobotController = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupTest()
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.EASY_STEPPING_STONES;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      assertTrue(simulationTestHelper.simulateNow(0.05));
      PlanarRegionsListMessage planarRegionsListMessage = createPlanarRegionsListMessage();
      simulationTestHelper.publishToController(planarRegionsListMessage);

      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      double z = getForcePointOffsetZInChestFrame();
      pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationSession().getTime(),
                                                        simulationTestHelper.getRobot(),
                                                        fullRobotModel.getChest().getParentJoint().getName(),
                                                        new Vector3D(0, 0, z));

      setupCameraForWalkingOverEasySteppingStones();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<FootControlModule.ConstraintType> footConstraintType = (YoEnum<FootControlModule.ConstraintType>) simulationTestHelper.findVariable(sidePrefix
               + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("WalkingHighLevelHumanoidController",
                                                                                                                    "walkingCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      simulationTestHelper.addYoGraphicDefinition(pushRobotController.getForceVizDefinition());

      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      double transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      totalMass = fullRobotModel.getTotalMass();

      assertTrue(simulationTestHelper.simulateNow(0.25));

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOverEasySteppingStones(swingTime, transferTime);
      footstepDataList.setAreFootstepsAdjustable(true);
      simulationTestHelper.publishToController(footstepDataList);
      assertTrue(simulationTestHelper.simulateNow(1.0));
   }

   @Test
   public void testWalkingOverSteppingStonesForwardPush()
   {
      setupTest();
      double transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOverEasySteppingStones(swingTime, transferTime);
      footstepDataList.setAreFootstepsAdjustable(true);
      simulationTestHelper.publishToController(footstepDataList);

      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;
      Vector3D firstForceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double percentWeight = 0.35;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      double stepDuration = swingTime + transferTime;
      boolean success = simulationTestHelper.simulateNow(footstepDataList.getFootstepDataList().size() * stepDuration + 1.5);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);
      //      simulationTestHelper.checkNothingChanged();

      Point3D center = new Point3D(-10.241987629532595, -0.8330256660954483, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }

   private void setupCameraForWalkingOverEasySteppingStones()
   {
      Point3D cameraFix = new Point3D(-8.6, -0.1, 0.94);
      Point3D cameraPosition = new Point3D(-14.0, -5.0, 2.7);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingOverEasySteppingStones(double swingTime, double transferTime)
   {
      List<Pose3D> footstepPoses = new ArrayList<>();
      footstepPoses.add(new Pose3D(new Point3D(-7.75, -0.55, 0.3), new Quaternion(0.0, 0.0, 1.0, 0.0)));
      footstepPoses.add(new Pose3D(new Point3D(-8.25, -0.95, 0.3), new Quaternion(0.0, 0.0, 1.0, 0.0)));
      footstepPoses.add(new Pose3D(new Point3D(-8.75, -0.55, 0.3), new Quaternion(0.0, 0.0, 1.0, 0.0)));
      footstepPoses.add(new Pose3D(new Point3D(-9.25, -0.95, 0.3), new Quaternion(0.0, 0.0, 1.0, 0.0)));
      footstepPoses.add(new Pose3D(new Point3D(-9.75, -0.55, 0.3), new Quaternion(0.0, 0.0, 1.0, 0.0)));
      footstepPoses.add(new Pose3D(new Point3D(-10.25, -1.00, 0.3), new Quaternion(0.0, 0.0, 1.0, 0.0)));
      footstepPoses.add(new Pose3D(new Point3D(-10.25, -0.65, 0.3), new Quaternion(0.0, 0.0, 1.0, 0.0)));

      for (Pose3D footstepPose : footstepPoses) // The footsteps were originally written in terms of desired ankle pose for Atlas, this transforms it to desired sole pose.
         footstepPose.appendTranslation(0.025, 0.0, -0.084);

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses, swingTime, transferTime);
   }

   private PlanarRegionsListMessage createPlanarRegionsListMessage()
   {
      List<Point3D> locations = new ArrayList<>();
      locations.add(new Point3D(-7.75, -0.55, 0.3));
      locations.add(new Point3D(-8.25, -0.95, 0.3));
      locations.add(new Point3D(-8.75, -0.55, 0.3));
      locations.add(new Point3D(-9.25, -0.95, 0.3));
      locations.add(new Point3D(-9.75, -0.55, 0.3));
      locations.add(new Point3D(-10.25, -1.0, 0.3));
      locations.add(new Point3D(-10.25, -0.65, 0.3));

      List<PlanarRegion> planarRegions = new ArrayList<>();
      int idStart = 10;
      for (int i = 0; i < locations.size() - 2; i++)
      {
         PlanarRegion planarRegion = createSteppingStonePlanarRegion(locations.get(i));
         planarRegion.setRegionId(idStart + i);
         planarRegions.add(planarRegion);
      }

      PlanarRegion platform = createEndPlanarRegion(locations.get(locations.size() - 2));
      platform.setRegionId(idStart + locations.size() - 2);

      planarRegions.add(platform);
      planarRegions.add(platform);

      List<PlanarRegionMessage> planarRegionsAsMessages = new ArrayList<>();
      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion planarRegion = planarRegions.get(i);
         if (planarRegion != null)
            planarRegionsAsMessages.add(PlanarRegionMessageConverter.convertToPlanarRegionMessage(planarRegion));
      }

      PlanarRegionsListMessage messageList = PlanarRegionMessageConverter.createPlanarRegionsListMessage(planarRegionsAsMessages);

      return messageList;
   }

   private PlanarRegion createSteppingStonePlanarRegion(Point3D centered)
   {
      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
      List<Point2D> points = createSteppingStoneFace();
      for (Point2D point : points)
      {
         point.add(centered.getX(), centered.getY());
         convexPolygon2D.addVertex(point);
      }
      convexPolygon2D.update();

      TranslationReferenceFrame planarRegionFrame = new TranslationReferenceFrame("planarRegionFrame", ReferenceFrame.getWorldFrame());
      planarRegionFrame.updateTranslation(new Vector3D(0.0, 0.0, 0.3));

      PlanarRegion planarRegion = new PlanarRegion(planarRegionFrame.getTransformToWorldFrame(), convexPolygon2D);
      return planarRegion;
   }

   private PlanarRegion createEndPlanarRegion(Point3D centered)
   {
      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
      List<Point2D> points = createPlatformFace();
      for (Point2D point : points)
      {
         point.add(centered.getX(), centered.getY());
         convexPolygon2D.addVertex(point);
      }
      convexPolygon2D.update();

      TranslationReferenceFrame planarRegionFrame = new TranslationReferenceFrame("planarRegionFrame", ReferenceFrame.getWorldFrame());
      planarRegionFrame.updateTranslation(new Vector3D(0.0, 0.0, 0.3));

      PlanarRegion planarRegion = new PlanarRegion(planarRegionFrame.getTransformToWorldFrame(), convexPolygon2D);
      return planarRegion;
   }

   private List<Point2D> createSteppingStoneFace()
   {
      ArrayList<Point2D> points = new ArrayList<>();
      points.add(new Point2D(0.25, 0.25));
      points.add(new Point2D(-0.25, 0.25));
      points.add(new Point2D(-0.25, -0.25));
      points.add(new Point2D(0.25, -0.25));

      return points;
   }

   private List<Point2D> createPlatformFace()
   {
      ArrayList<Point2D> points = new ArrayList<>();
      points.add(new Point2D(0.25, 1.0));
      points.add(new Point2D(-0.25, 1.0));
      points.add(new Point2D(-0.25, -1.0));
      points.add(new Point2D(0.25, -1.0));

      return points;
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<FootControlModule.ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<FootControlModule.ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean testCondition(double time)
      {
         return footConstraintType.getEnumValue() == FootControlModule.ConstraintType.SWING;
      }
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(YoEnum<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean testCondition(double time)
      {
         if (side == RobotSide.LEFT)
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_LEFT_SUPPORT);
         }
         else
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT);
         }
      }
   }

}

package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoEnum;

@Tag("humanoid-obstacle-3")
public abstract class AvatarPushRecoveryOverSteppingStonesTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private PushRobotController pushRobotController;

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

      if (pushRobotController != null)
      {
         pushRobotController = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.EASY_STEPPING_STONES;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.05));
      PlanarRegionsListMessage planarRegionsListMessage = createPlanarRegionsListMessage();
      drcSimulationTestHelper.publishToController(planarRegionsListMessage);

      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      double z = getForcePointOffsetZInChestFrame();
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel.getChest().getParentJoint().getName(),
                                                    new Vector3D(0, 0, z));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      setupCameraForWalkingOverEasySteppingStones(scs);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<FootControlModule.ConstraintType> footConstraintType = (YoEnum<FootControlModule.ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      scs.addYoGraphic(pushRobotController.getForceVisualizer());


      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      double transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      totalMass = fullRobotModel.getTotalMass();

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25));

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOverEasySteppingStones(swingTime, transferTime);
      footstepDataList.setAreFootstepsAdjustable(true);
      drcSimulationTestHelper.publishToController(footstepDataList);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
   }



	@Test
   public void testWalkingOverSteppingStonesForwardPush() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      double transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();


      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOverEasySteppingStones(swingTime, transferTime);
      footstepDataList.setAreFootstepsAdjustable(true);
      drcSimulationTestHelper.publishToController(footstepDataList);

      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;
      Vector3D firstForceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double percentWeight = 0.35;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      double stepDuration = swingTime + transferTime;
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footstepDataList.getFootstepDataList().size() * stepDuration + 1.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      Point3D center = new Point3D(-10.241987629532595, -0.8330256660954483, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }

 
   private void setupCameraForWalkingOverEasySteppingStones(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(-8.6, -0.1, 0.94);
      Point3D cameraPosition = new Point3D(-14.0, -5.0, 2.7);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForWalkingOverEasySteppingStones(double swingTime, double transferTime)
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      List<Point3D> locations = new ArrayList<>();
      locations.add(new Point3D(-7.75, -0.55, 0.3));
      locations.add(new Point3D(-8.25, -0.95, 0.3));
      locations.add(new Point3D(-8.75, -0.55, 0.3));
      locations.add(new Point3D(-9.25, -0.95, 0.3));
      locations.add(new Point3D(-9.75, -0.55, 0.3));
      locations.add(new Point3D(-10.25, -1.0, 0.3));
      locations.add(new Point3D(-10.25, -0.65, 0.3));

      List<Quaternion> orientations = new ArrayList<>();
      orientations.add(new Quaternion(0.0, 0.0, 1.0, 0.0));
      orientations.add(new Quaternion(0.0, 0.0, 1.0, 0.0));
      orientations.add(new Quaternion(0.0, 0.0, 1.0, 0.0));
      orientations.add(new Quaternion(0.0, 0.0, 1.0, 0.0));
      orientations.add(new Quaternion(0.0, 0.0, 1.0, 0.0));
      orientations.add(new Quaternion(0.0, 0.0, 1.0, 0.0));
      orientations.add(new Quaternion(0.0, 0.0, 1.0, 0.0));

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, locations.size());
      for (int i = 0; i < locations.size(); i++)
      {
         FramePoint3D placeToStep = new FramePoint3D(ReferenceFrame.getWorldFrame(), locations.get(i));
         FootstepDataMessage data = createFootstepDataMessage(robotSides[i], placeToStep, orientations.get(i));
         message.getFootstepDataList().add().set(data);
      }

      return message;
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

   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint3D placeToStep, Quaternion orientation)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint3D placeToStepInWorld = new FramePoint3D(placeToStep);
      placeToStepInWorld.changeFrame(ReferenceFrame.getWorldFrame());

      footstepData.getLocation().set(placeToStepInWorld);
      footstepData.getOrientation().set(orientation);
      footstepData.setRobotSide(robotSide.toByte());

      return footstepData;
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

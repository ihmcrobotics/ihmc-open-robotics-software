package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import controller_msgs.msg.dds.StepConstraintsListMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.stepAdjustment.SteppableRegionsCalculator;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintMessageConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransformGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.tools.TerrainObjectDefinitionTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

public abstract class AvatarPushRecoveryOverGapTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
//   private StepConstraintToolboxModule stepConstraintModule;

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private PushRobotControllerSCS2 pushRobotController;

   private double swingTime, transferTime;

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }

   public void setupTest(double platformWidth)
   {
      double platform1Length = 0.7;
      double platform2Length = 1.0;
      double gapWidth = 0.10;
      double sideGapWidth = 0.04;

      GapPlanarRegionEnvironment environment = new GapPlanarRegionEnvironment(platform1Length, platform2Length, platformWidth, gapWidth, sideGapWidth);

      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ((YoBoolean) simulationTestHelper.findVariable("switchPlanarRegionConstraintsAutomatically")).set(true);

      SteppableRegionsCalculator stepConstraintCalculator = new SteppableRegionsCalculator(4.0, simulationTestHelper.getControllerRegistry());

      PlanarRegionsList planarRegionsList = environment.getPlanarRegionsList();

      stepConstraintCalculator.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());
      List<StepConstraintRegion> stepConstraints = stepConstraintCalculator.computeSteppableRegions();

      double z = getForcePointOffsetZInChestFrame();
      pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                        simulationTestHelper.getRobot(),
                                                        robotModel.createFullRobotModel().getChest().getParentJoint().getName(),
                                                        new Vector3D(0, 0, z));

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

      simulationTestHelper.setCameraPosition(8.0, -8.0, 5.0);
      simulationTestHelper.setCameraFocusPosition(1.5, 0.0, 0.8);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();

      swingTime = walkingControllerParameters.getDefaultSwingTime();
      transferTime = walkingControllerParameters.getDefaultTransferTime();

      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateNow(0.5));

      FootstepDataListMessage footsteps = createFootstepDataListMessage(swingTime, transferTime);
      StepConstraintsListMessage constraintsListMessage = StepConstraintMessageConverter.convertToStepConstraintsListMessage(stepConstraints);
      footsteps.getDefaultStepConstraints().set(constraintsListMessage);
      simulationTestHelper.publishToController(footsteps);

   }

   @Test
   public void testNoPush()
   {
      setupTest(0.5);

      double simulationTime = (swingTime + transferTime) * 4 + 1.0;
      assertTrue(simulationTestHelper.simulateNow(simulationTime));

      Point3D center = new Point3D(1.05, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   @Test
   public void testForwardPush()
   {
      setupTest(0.75);

      simulationTestHelper.simulateNow(1.0);
      double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;
      Vector3D firstForceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentWeight = 0.5;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      double simulationTime = (swingTime + transferTime) * 4 + 10.0;
      boolean success = simulationTestHelper.simulateNow(simulationTime);

      Point3D center = new Point3D(1.05, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.4, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      assertTrue(success);
   }

   @Test
   public void testSidePush()
   {
      setupTest(0.5);

      double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;
      Vector3D firstForceDirection = new Vector3D(0.0, 1.0, 0.0);
      double percentWeight = 0.3;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      double simulationTime = (swingTime + transferTime) * 4 + 1.0;
      assertTrue(simulationTestHelper.simulateNow(simulationTime));

      Point3D center = new Point3D(1.05, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   private FootstepDataListMessage createFootstepDataListMessage(double swingTime, double transferTime)
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.setAreFootstepsAdjustable(true);
      Point3D location = new Point3D(0.3, 0.15, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(0.75, -0.15, 0.0);
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(1.05, 0.15, 0.0);
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));
      location = new Point3D(1.05, -0.15, 0.0);
      message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      return message;
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
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

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   private static class GapPlanarRegionEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final RigidBodyTransformGenerator transformGenerator = new RigidBodyTransformGenerator();
      private final TerrainObjectDefinition terrainObjectDefinition ;
      private final CombinedTerrainObject3D combinedTerrainObject;
      private final PlanarRegionsList planarRegionsList;
      private int id = 0;

      public GapPlanarRegionEnvironment(double platform1Length, double platform2Length, double platformWidth, double forwardGapSize, double sideGapSize)
      {
         double platformHeight = 0.2;
         transformGenerator.translate(0.0, 0.0, 0.01);
         combinedTerrainObject = new CombinedTerrainObject3D("gapEnvironment");
         planarRegionsList = new PlanarRegionsList();

         addBox(platform1Length, platformWidth, platformHeight);

         double platform2Center = 0.5 * (platform1Length + platform2Length) + forwardGapSize;
         transformGenerator.translate(0.5 * (platform1Length + platform2Length) + forwardGapSize, 0.0, 0.0);
         addBox(platform2Length, platformWidth, platformHeight);

         double sideWidth = 0.18;
         double sideLength = platform1Length + platform2Length + forwardGapSize;
         double distanceToCenter = 0.5 * sideLength - 0.5 * platform1Length;
         transformGenerator.translate(-platform2Center + distanceToCenter, 0.5 * platformWidth + sideGapSize + 0.5 * sideWidth, 0.0);
         addBox(sideLength, sideWidth, platformHeight);

         terrainObjectDefinition = TerrainObjectDefinitionTools.toTerrainObjectDefinition(this);
      }

      void addBox(double length, double width, double height)
      {
         RigidBodyTransform boxCenter = transformGenerator.getRigidBodyTransformCopy();
         boxCenter.appendTranslation(0.0, 0.0, -0.5 * height);

         RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(boxCenter, length, width, height), YoAppearance.DarkGray());
         combinedTerrainObject.addTerrainObject(newBox);

         addPolygon(createRectanglePolygon(length, width));
      }

      private static ConvexPolygon2D createRectanglePolygon(double lengthX, double widthY)
      {
         ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
         convexPolygon.addVertex(lengthX / 2.0, widthY / 2.0);
         convexPolygon.addVertex(-lengthX / 2.0, widthY / 2.0);
         convexPolygon.addVertex(-lengthX / 2.0, -widthY / 2.0);
         convexPolygon.addVertex(lengthX / 2.0, -widthY / 2.0);
         convexPolygon.update();
         return convexPolygon;
      }

      private void addPolygon(ConvexPolygon2D polygon)
      {
         PlanarRegion planarRegion = new PlanarRegion(transformGenerator.getRigidBodyTransformCopy(), polygon);
         planarRegion.setRegionId(id++);
         planarRegionsList.addPlanarRegion(planarRegion);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return combinedTerrainObject;
      }

      public PlanarRegionsList getPlanarRegionsList()
      {
         return planarRegionsList;
      }
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

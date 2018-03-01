package us.ihmc.avatar.roughTerrainWalking;

import org.junit.After;
import org.junit.Before;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoEnum;

import static junit.framework.TestCase.assertTrue;

public abstract class AvatarPushRecoveryOverGapTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private PushRobotController pushRobotController;

   private double swingTime, transferTime;

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }

   public void setupTest() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();

      double platform1Length = 0.7;
      double platform2Length = 1.0;
      double gapWidth = 0.10;
      double sideGapWidth = 0.04;

      GapPlanarRegionEnvironment environment = new GapPlanarRegionEnvironment(platform1Length, platform2Length, 0.55, gapWidth, sideGapWidth);

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.createSimulation(className);

      PlanarRegionsList planarRegionsList = environment.getPlanarRegionsList();
      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
      planarRegionsListMessage.setUniqueId(5L);

      drcSimulationTestHelper.getControllerCommunicator().attachListener(RequestPlanarRegionsListMessage.class, new PacketConsumer<RequestPlanarRegionsListMessage>()
      {
         @Override
         public void receivedPacket(RequestPlanarRegionsListMessage packet)
         {
            drcSimulationTestHelper.send(planarRegionsListMessage);
         }
      });

      double z = getForcePointOffsetZInChestFrame();
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), robotModel.createFullRobotModel().getChest().getParentJoint().getName(),
            new Vector3D(0, 0, z));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<FootControlModule.ConstraintType> footConstraintType = (YoEnum<FootControlModule.ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule", footPrefix + "State");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(8.0, -8.0, 5.0);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(1.5, 0.0, 0.8);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();

      swingTime = walkingControllerParameters.getDefaultSwingTime();
      transferTime = walkingControllerParameters.getDefaultTransferTime();

      FootstepDataListMessage footsteps = createFootstepDataListMessage(swingTime, transferTime);
      drcSimulationTestHelper.send(footsteps);
      drcSimulationTestHelper.send(planarRegionsListMessage);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
   }


   public void testNoPush() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double simulationTime = (swingTime + transferTime) * 4 + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(1.05, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testForwardPush() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double totalMass  = getRobotModel().createFullRobotModel().getTotalMass();
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;
      Vector3D firstForceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentWeight = 0.5;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      double simulationTime = (swingTime + transferTime) * 4 + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(1.05, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testSidePush() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      double totalMass  = getRobotModel().createFullRobotModel().getTotalMass();
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double delay = 0.5 * swingTime;
      Vector3D firstForceDirection = new Vector3D(0.0, 1.0, 0.0);
      double percentWeight = 0.3;
      double magnitude = percentWeight * totalMass * 9.81;
      double duration = 0.1;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      double simulationTime = (swingTime + transferTime) * 4 + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(1.05, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   private FootstepDataListMessage createFootstepDataListMessage(double swingTime, double transferTime)
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      Point3D location = new Point3D(0.3, 0.15, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(0.75, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(1.05, 0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));
      location = new Point3D(1.05, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      return message;
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

      if (pushRobotController != null)
      {
         pushRobotController = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private class GapPlanarRegionEnvironment extends PlanarRegionEnvironmentInterface
   {
      public GapPlanarRegionEnvironment(double platform1Length, double platform2Length, double platformWidth, double forwardGapSize, double sideGapSize)
      {
         generator.translate(0.0, 0.0, -0.01);
         generator.addCubeReferencedAtBottomMiddle(platform1Length, platformWidth, 0.01); // ground

         double platform2Center = 0.5 * (platform1Length + platform2Length) + forwardGapSize;
         generator.translate(0.5 * (platform1Length + platform2Length) + forwardGapSize, 0.0, 0.0);
         generator.addCubeReferencedAtBottomMiddle(platform2Length, platformWidth, 0.01); // ground

         double sideWidth = 0.18;
         double sideLength = platform1Length + platform2Length + forwardGapSize;
         double distanceToCenter = 0.5 * sideLength - 0.5 * platform1Length;
         generator.translate(-platform2Center + distanceToCenter, 0.5 * platformWidth  + sideGapSize + 0.5 * sideWidth, 0.0);
         generator.addCubeReferencedAtBottomMiddle(sideLength, sideWidth, 0.01); // ground
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
      public boolean checkCondition()
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
      public boolean checkCondition()
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

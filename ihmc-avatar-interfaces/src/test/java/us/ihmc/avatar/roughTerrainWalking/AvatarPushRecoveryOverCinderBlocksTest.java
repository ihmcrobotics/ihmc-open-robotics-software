package us.ihmc.avatar.roughTerrainWalking;

import org.junit.After;
import org.junit.Before;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
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
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.CinderBlockFieldPlanarRegionEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoEnum;

import static junit.framework.TestCase.assertTrue;

public abstract class AvatarPushRecoveryOverCinderBlocksTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final double cinderBlockTiltDegrees = 15;
   private static final double cinderBlockTiltRadians = Math.toRadians(cinderBlockTiltDegrees);

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private PushRobotController pushRobotController;

   private double swingTime, transferTime;

   protected double getForcePointOffsetZInChestFrame()
   {
      return 0.3;
   }

   public int setUpFlatBlockTest() throws SimulationExceededMaximumTimeException
   {
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup();
      PlanarRegionsListMessage planarRegionsListMessage = setUpTest(startingLocation);

      FootstepDataListMessage footsteps = createFlatBlocksFootstepDataListMessage(swingTime, transferTime);
      drcSimulationTestHelper.send(footsteps);
      drcSimulationTestHelper.send(planarRegionsListMessage);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      return footsteps.size();
   }

   public int setUpForwardFlatBlockTest() throws SimulationExceededMaximumTimeException
   {
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup();
      PlanarRegionsListMessage planarRegionsListMessage = setUpTest(startingLocation);

      FootstepDataListMessage footsteps = createFlatBlocksForwardFootstepDataListMessage(swingTime, transferTime);
      footsteps.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(footsteps);
      drcSimulationTestHelper.send(planarRegionsListMessage);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      return footsteps.size();
   }

   public int setUpTiltedBlockTest() throws SimulationExceededMaximumTimeException
   {
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup(3.5, 0.0, 0.0);
      PlanarRegionsListMessage planarRegionsListMessage = setUpTest(startingLocation);

      FootstepDataListMessage footsteps = createTiltedBlocksFootstepDataListMessage(swingTime, transferTime);
      drcSimulationTestHelper.send(footsteps);
      drcSimulationTestHelper.send(planarRegionsListMessage);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      return footsteps.size();
   }

   public int setUpForwardTiltedBlockTest() throws SimulationExceededMaximumTimeException
   {
      OffsetAndYawRobotInitialSetup startingLocation = new OffsetAndYawRobotInitialSetup(3.5, 0.0, 0.0);
      PlanarRegionsListMessage planarRegionsListMessage = setUpTest(startingLocation);

      FootstepDataListMessage footsteps = createTiltedBlocksForwardFootstepDataListMessage(swingTime, transferTime);
      drcSimulationTestHelper.send(footsteps);
      drcSimulationTestHelper.send(planarRegionsListMessage);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      return footsteps.size();
   }


   public PlanarRegionsListMessage setUpTest(OffsetAndYawRobotInitialSetup startingLocation) throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();

      CinderBlockFieldPlanarRegionEnvironment environment = new CinderBlockFieldPlanarRegionEnvironment();

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setTestEnvironment(environment);
      drcSimulationTestHelper.setStartingLocation(startingLocation);
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

      Point3D cameraPosition = new Point3D(8.0, -8.0, 5.0);
      Point3D cameraFix = new Point3D(1.5, 0.0, 0.8);
      cameraPosition.add(startingLocation.getAdditionalOffset());
      cameraFix.add(startingLocation.getAdditionalOffset());
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraPosition(cameraPosition);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraFix(cameraFix);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();

      swingTime = walkingControllerParameters.getDefaultSwingTime();
      transferTime = walkingControllerParameters.getDefaultTransferTime();

      return planarRegionsListMessage;
   }



   public void testNoPushFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpFlatBlockTest();

      double simulationTime = (swingTime + transferTime) * numberOfSteps + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(3.1, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testNoPushForwardWalkOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpForwardFlatBlockTest();

      double simulationTime = (swingTime + transferTime) * numberOfSteps + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(3.1, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testNoPushTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpTiltedBlockTest();

      double simulationTime = (swingTime + transferTime) * numberOfSteps + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(7.3, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testNoPushForwardTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpForwardTiltedBlockTest();

      double simulationTime = (swingTime + transferTime) * numberOfSteps + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(7.3, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testPushOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpFlatBlockTest();

      double totalMass  = getRobotModel().createFullRobotModel().getTotalMass();

      int stepsTaken = 0;
      double simulationTime = (swingTime + transferTime) * 2.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken = stepsTaken + 2;

      // push on the third step
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double duration = 0.1;
      double delay = 0.5 * swingTime - duration;
      Vector3D firstForceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentWeight = 0.3;
      double magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken++;

      // push on fourth step
      StateTransitionCondition secondPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      duration = 0.1;
      delay = 0.5 * swingTime - duration;
      firstForceDirection = new Vector3D(1.0, 0.0, 0.0);
      percentWeight = 0.4;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(secondPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * 2.5;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      stepsTaken += 3;

      // push on seventh step
      StateTransitionCondition thirdPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      duration = 0.1;
      delay = 0.5 * swingTime - duration;
      firstForceDirection = new Vector3D(0.0, 1.0, 0.0);
      percentWeight = 0.5;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(thirdPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      stepsTaken++;

      // push on eighth step
      StateTransitionCondition fourthPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      duration = 0.1;
      delay = 0.5 * swingTime - duration;
      firstForceDirection = new Vector3D(1.0, 0.0, 0.0);
      percentWeight = 0.4;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(fourthPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      stepsTaken++;

      // push on ninth step
      StateTransitionCondition fifthPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      duration = 0.1;
      delay = 0.5 * swingTime - duration;
      firstForceDirection = new Vector3D(1.0, 0.0, 0.0);
      percentWeight = 0.45;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(fifthPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * 2;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      stepsTaken += 2;

      // push on eleventh step
      StateTransitionCondition sixthPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      duration = 0.1;
      delay = 0.5 * swingTime - duration;
      firstForceDirection = new Vector3D(1.0, -1.0, 0.0);
      percentWeight = 0.6;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(sixthPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * (numberOfSteps - stepsTaken) + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(7.3, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testForwardPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpForwardFlatBlockTest();
      double totalMass  = getRobotModel().createFullRobotModel().getTotalMass();

      int stepsTaken = 0;
      double simulationTime = (swingTime + transferTime) * 3.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken += 3;

      // push on the fourth step
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double duration = 0.1;
      double delay = 0.5 * swingTime - duration;
      Vector3D firstForceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentWeight = 0.5;
      double magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * (numberOfSteps - stepsTaken) + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(3.1, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testLeftSidewaysPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpForwardFlatBlockTest();
      double totalMass  = getRobotModel().createFullRobotModel().getTotalMass();

      int stepsTaken = 0;
      double simulationTime = (swingTime + transferTime) * 4.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken += 4;

      // push on the fifth step
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      double duration = 0.1;
      double delay = 0.5 * swingTime - duration;
      Vector3D firstForceDirection = new Vector3D(0.0, 1.0, 0.0);
      double percentWeight = 0.5;
      double magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * (numberOfSteps - stepsTaken) + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(3.1, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testRightSidewaysPushWalkWithOffsetOverFlatBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpForwardFlatBlockTest();
      double totalMass  = getRobotModel().createFullRobotModel().getTotalMass();

      int stepsTaken = 0;
      double simulationTime = (swingTime + transferTime) * 3.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken += 3;

      // push on the fourth step
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double duration = 0.2;
      double delay = 0.5 * swingTime - duration;
      Vector3D firstForceDirection = new Vector3D(0.0, -1.0, 0.0);
      double percentWeight = 0.25;
      double magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, firstForceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * (numberOfSteps - stepsTaken) + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(3.1, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   public void testPushOverTiltedBlocks() throws SimulationExceededMaximumTimeException
   {
      int numberOfSteps = setUpTiltedBlockTest();

      double totalMass  = getRobotModel().createFullRobotModel().getTotalMass();

      int stepsTaken = 0;
      double simulationTime = (swingTime + transferTime) * 3.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken += 3;

      // push on the fourth step
      StateTransitionCondition firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double duration = 0.1;
      double delay = 0.5 * swingTime - duration;
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentWeight = 0.55;
      double magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, forceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * 2;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken += 2;

      // push on the sixth step
      firstPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      duration = 0.2;
      delay = 0.5 * swingTime - 0.05 - duration;
      forceDirection = new Vector3D(1.0, 0.0, 0.0);
      percentWeight = 0.3;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(firstPushCondition, delay, forceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      stepsTaken++;

      // push on seventh step
      StateTransitionCondition secondPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      duration = 0.2;
      delay = 0.5 * swingTime - 0.05 - duration;
      forceDirection = new Vector3D(-0.2, 1.0, 0.0);
      percentWeight = 0.25;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(secondPushCondition, delay, forceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      stepsTaken += 1;

      // push on eighth step
      StateTransitionCondition thirdPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      duration = 0.2;
      delay = 0.5 * swingTime - 0.05 - duration;
      forceDirection = new Vector3D(1.0, 0.0, 0.0);
      percentWeight = 0.25;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(thirdPushCondition, delay, forceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * 2.5;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      stepsTaken += 3;

      // push on eleventh step
      StateTransitionCondition fourthPushCondition = singleSupportStartConditions.get(RobotSide.LEFT);
      duration = 0.25;
      delay = 0.5 * swingTime - 0.05 - duration;
      forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      percentWeight = 0.3;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(fourthPushCondition, delay, forceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * 3;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      stepsTaken += 3;

      // push on fourteenth step
      StateTransitionCondition fifthPushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      duration = 0.2;
      delay = Math.max(0.0, 0.5 * swingTime - 0.05 - duration);
      forceDirection = new Vector3D(0.0, -1.0, 0.0);
      percentWeight = 0.3;
      magnitude = percentWeight * totalMass * 9.81;
      pushRobotController.applyForceDelayed(fifthPushCondition, delay, forceDirection, magnitude, duration);

      simulationTime = (swingTime + transferTime) * (numberOfSteps - stepsTaken) + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      Point3D center = new Point3D(7.3, 0.0, 1.0893768421917251);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);
   }

   /*
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
   */

   private FootstepDataListMessage createFlatBlocksFootstepDataListMessage(double swingTime, double transferTime)
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);

      Point3D location = new Point3D(0.3, 0.15, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(0.4, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(0.7, 0.15, 0.08);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(0.7, -0.15, 0.08);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(1.1, 0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(1.1, -0.15, 0.16);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(1.5, 0.15, 0.08);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(1.5, -0.15, 0.16);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(1.9, 0.15, 0.16);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(2.3, -0.15, 0.16);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(0.22);
      message.add(footstep);

      /*
      location = new Point3D(2.3, 0.15, 0.08);
      message.add(new FootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(2.3, -0.15, 0.16);
      message.add(new FootstepDataMessage(RobotSide.RIGHT, location, orientation));
      */

      location = new Point3D(2.7, 0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(2.7, -0.15, 0.08);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(3.1, 0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(3.1, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      return message;
   }

   private FootstepDataListMessage createFlatBlocksForwardFootstepDataListMessage(double swingTime, double transferTime)
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      double swingHeight = 0.18;

      Point3D location = new Point3D(0.3, 0.15, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(0.4, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(0.7, 0.15, 0.08);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(1.1, -0.15, 0.08);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(1.5, 0.15, 0.0);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(1.9, -0.15, 0.16);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(2.3, 0.15, 0.08);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(2.7, -0.15, 0.16);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(3.1, 0.15, 0.0);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(3.1, -0.15, 0.0);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      message.add(footstep);

      return message;
   }

   private FootstepDataListMessage createTiltedBlocksFootstepDataListMessage(double swingTime, double transferTime)
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      Point3D location = new Point3D(3.7, 0.15, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);

      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(3.7, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(4.1, 0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendPitchRotation(-cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(4.1, -0.15, 0.2);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(4.54, 0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendPitchRotation(cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(4.54, -0.15, 0.2);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(4.98, 0.15, 0.21);
      orientation = new Quaternion();
      orientation.appendRollRotation(-cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(4.98, -0.15, 0.21);
      orientation = new Quaternion();
      orientation.appendRollRotation(cinderBlockTiltRadians);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(0.18);
      message.add(footstep);

      location = new Point3D(5.39, 0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendRollRotation(cinderBlockTiltRadians);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(0.18);
      message.add(footstep);

      location = new Point3D(5.39, -0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendRollRotation(-cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(5.8, 0.15, 0.3);
      orientation = new Quaternion();
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(5.8, -0.15, 0.3);
      orientation = new Quaternion();
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(6.225, 0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendPitchRotation(cinderBlockTiltRadians);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(0.18);
      message.add(footstep);

      location = new Point3D(6.225, -0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendRollRotation(-cinderBlockTiltRadians);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      //footstep.setSwingHeight(0.25);
      message.add(footstep);

      location = new Point3D(6.665, 0.15, 0.35);
      orientation = new Quaternion();
      orientation.appendRollRotation(cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(6.665, -0.15, 0.35);
      orientation = new Quaternion();
      orientation.appendPitchRotation(cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(7.08, 0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendPitchRotation(-cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(7.08, -0.15, 0.2);
      orientation = new Quaternion();
      orientation.appendRollRotation(cinderBlockTiltRadians);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(7.45, 0.15, 0.0);
      orientation = new Quaternion();
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(7.45, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      return message;
   }

   private FootstepDataListMessage createTiltedBlocksForwardFootstepDataListMessage(double swingTime, double transferTime)
   {
      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      Point3D location = new Point3D(3.7, 0.15, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      double swingHeight = 0.2;

      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

      location = new Point3D(3.7, -0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(4.1, 0.15, 0.2);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(4.54, -0.15, 0.2);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(4.98, 0.15, 0.21);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(5.39, -0.15, 0.2);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(5.8, 0.15, 0.3);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(6.225, -0.15, 0.3);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(6.225, 0.15, 0.35);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(6.665, -0.15, 0.35);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(7.08, 0.15, 0.35);
      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation);
      footstep.setSwingHeight(swingHeight);
      message.add(footstep);

      location = new Point3D(7.45, -0.15, 0.0);
      orientation = new Quaternion();
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, location, orientation));

      location = new Point3D(7.45, 0.15, 0.0);
      message.add(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, location, orientation));

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

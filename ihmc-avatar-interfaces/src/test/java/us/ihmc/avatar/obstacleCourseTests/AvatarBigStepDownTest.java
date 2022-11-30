package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.apache.commons.lang.mutable.MutableInt;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.DefaultSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.SplitFractionCalculatorParametersReadOnly;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarBigStepDownTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private static final double epsilon = 1.0e-6;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      simulationTestingParameters.setKeepSCSUp(simulationTestingParameters.getKeepSCSUp()
            && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testWalkingOffOfLargePlatform()
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_LARGE_PLATFORM;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(false);
      simulationTestHelperFactory.setUseRobotDefinitionCollisions(false);

      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start(false);
      ((YoBoolean) simulationTestHelper.getControllerRegistry().findVariable("doToeOffIfPossibleInSingleSupport")).set(true);

      Point3D cameraFix = new Point3D(-4.68, -7.8, 0.55);
      Point3D cameraPosition = new Point3D(-8.6, -4.47, 0.58);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      Quaternion footRotation = new Quaternion();
      footRotation.setToYawOrientation(selectedLocation.getStartingLocationOffset().getYaw());

      FootstepDataMessage firstStep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                     new Point3D(-5.8 + 0.15, -7.471 - 0.15, 0.05),
                                                                                     footRotation);
      FootstepDataMessage secondStep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                      new Point3D(-5.8 - 0.15, -7.471 + 0.15, 0.05),
                                                                                      footRotation);

      @SuppressWarnings("unchecked")
      YoEnum<FootControlModule.ConstraintType> leftFootState = ((YoEnum<FootControlModule.ConstraintType>) simulationTestHelper.findVariable("rightFootCurrentState"));

      simulationTestHelper.publishToController(HumanoidMessageTools.createFootstepDataListMessage(firstStep));

      success = success && simulationTestHelper.simulateNow(4.0);

      assertEquals(FootControlModule.ConstraintType.TOES, leftFootState.getEnumValue());

      MutableInt leftFootStateChanges = new MutableInt(0);
      leftFootState.addListener(v -> leftFootStateChanges.increment());

      simulationTestHelper.publishToController(HumanoidMessageTools.createFootstepDataListMessage(secondStep));

      success = success && simulationTestHelper.simulateNow(4.0);

      assertEquals(2, leftFootStateChanges.getValue());

      assertTrue(success);

      Point3D center = new Point3D(-5.8, -7.5, 0.87);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.2);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSplitFractionInBigStepDown()
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_LARGE_PLATFORM;

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(false);
      simulationTestHelperFactory.setUseRobotDefinitionCollisions(false);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start(false);

      ((YoBoolean) simulationTestHelper.getControllerRegistry().findVariable("doToeOffIfPossibleInSingleSupport")).set(true);

      Point3D cameraFix = new Point3D(-4.68, -7.8, 0.55);
      Point3D cameraPosition = new Point3D(-8.6, -4.47, 0.58);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      @SuppressWarnings("unchecked")
      YoEnum<FootControlModule.ConstraintType> rightFootState = ((YoEnum<FootControlModule.ConstraintType>) simulationTestHelper.findVariable("rightFootCurrentState"));
      @SuppressWarnings("unchecked")
      YoEnum<WalkingStateEnum> currentWalkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("walkingCurrentState");

      YoDouble processedSplitFraction = (YoDouble) simulationTestHelper.findVariable("processedFinalTransferSplitFraction");
      YoDouble processedWeightDistribution = (YoDouble) simulationTestHelper.findVariable("processedFinalTransferWeightDistribution");
      YoDouble percentageWeightOnLeftFoot = (YoDouble) simulationTestHelper.findVariable("percentageStandingWeightDistributionOnLeftFoot");

      // check that the split fraction parameters change in the correct walking states
      SplitFractionCalculatorParametersReadOnly defaultSplitFractions = new DefaultSplitFractionCalculatorParameters();
      processedSplitFraction.addListener(v -> checkSplitFractionParameters(currentWalkingState,
                                                                           rightFootState,
                                                                           processedSplitFraction,
                                                                           defaultSplitFractions.getTransferSplitFractionAtFullDepth()));
      processedWeightDistribution.addListener(v -> checkSplitFractionParameters(currentWalkingState,
                                                                                rightFootState,
                                                                                processedWeightDistribution,
                                                                                defaultSplitFractions.getTransferWeightDistributionAtFullDepth()));
      percentageWeightOnLeftFoot.addListener(v -> checkPercentageWeightParameters(currentWalkingState,
                                                                                  rightFootState,
                                                                                  percentageWeightOnLeftFoot,
                                                                                  1.0 - defaultSplitFractions.getTransferWeightDistributionAtFullDepth()));

      Quaternion footRotation = new Quaternion();
      footRotation.setToYawOrientation(selectedLocation.getStartingLocationOffset().getYaw());

      FootstepDataMessage firstStep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                     new Point3D(-5.8 + 0.15, -7.471 - 0.15, 0.0),
                                                                                     footRotation);
      FootstepDataMessage secondStep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                      new Point3D(-5.8 - 0.15, -7.471 + 0.15, 0.0),
                                                                                      footRotation);

      simulationTestHelper.publishToController(HumanoidMessageTools.createFootstepDataListMessage(firstStep));

      success = success && simulationTestHelper.simulateNow(4.0);

      simulationTestHelper.publishToController(HumanoidMessageTools.createFootstepDataListMessage(secondStep));

      success = success && simulationTestHelper.simulateNow(4.0);

      assertTrue(success);

      Point3D center = new Point3D(-5.8, -7.5, 0.87);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.2);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void checkSplitFractionParameters(YoEnum<WalkingStateEnum> currentWalkingState,
                                             YoEnum<FootControlModule.ConstraintType> rightFootCurrentState,
                                             YoDouble splitFraction,
                                             double splitFractionValue)
   {
      if (MathTools.epsilonEquals(splitFraction.getDoubleValue(), splitFractionValue, epsilon))
      { // period during which split fractions are not the default ones
         boolean success = currentWalkingState.getEnumValue().equals(WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT)
               || currentWalkingState.getEnumValue().equals(WalkingStateEnum.WALKING_RIGHT_SUPPORT)
               || (currentWalkingState.getEnumValue().equals(WalkingStateEnum.TO_STANDING)
                     && rightFootCurrentState.getEnumValue().equals(FootControlModule.ConstraintType.TOES));
         assertTrue(success);
      }
      else if (MathTools.epsilonEquals(splitFraction.getDoubleValue(), 0.5, epsilon))
      { // changing to default split fractions must not occur when in transfer to standing
         boolean success = !currentWalkingState.getEnumValue().equals(WalkingStateEnum.TO_STANDING);
         assertTrue(success);
      }
   }

   private void checkPercentageWeightParameters(YoEnum<WalkingStateEnum> currentWalkingState,
                                                YoEnum<FootControlModule.ConstraintType> rightFootCurrentState,
                                                YoDouble percentageWeightDistribution,
                                                double splitFractionValue)
   {
      if (MathTools.epsilonEquals(percentageWeightDistribution.getDoubleValue(), splitFractionValue, epsilon))
      { // period during which percent weight distribution is not the default
         boolean success = currentWalkingState.getEnumValue().equals(WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT)
               || currentWalkingState.getEnumValue().equals(WalkingStateEnum.WALKING_RIGHT_SUPPORT)
               || rightFootCurrentState.getEnumValue().equals(FootControlModule.ConstraintType.TOES)
                     && (currentWalkingState.getEnumValue().equals(WalkingStateEnum.TO_STANDING)
                           || currentWalkingState.getEnumValue().equals(WalkingStateEnum.STANDING));
         assertTrue(success);
      }
      else if (MathTools.epsilonEquals(percentageWeightDistribution.getDoubleValue(), 0.5, epsilon))
      { // changing to default split fractions must not occur when transferring to take "square up" step
         boolean success = currentWalkingState.getEnumValue().equals(WalkingStateEnum.TO_WALKING_LEFT_SUPPORT)
               || currentWalkingState.getEnumValue().equals(WalkingStateEnum.WALKING_LEFT_SUPPORT)
               || (rightFootCurrentState.getEnumValue().equals(FootControlModule.ConstraintType.FULL)
                     && currentWalkingState.getEnumValue().equals(WalkingStateEnum.TO_STANDING)
                     || currentWalkingState.getEnumValue().equals(WalkingStateEnum.STANDING));
         assertTrue(success);
      }
   }

}

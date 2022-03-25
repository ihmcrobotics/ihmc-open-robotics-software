package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarFeetErrorTranslationTest implements MultiRobotTestInterface
{
   private static final int numberOfSteps = 10;
   private static final double stepLength = 0.25;
   private static final double stepWidth = 0.25;

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testForwardWalk()
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      LogTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      DRCRobotModel robotModel = getRobotModel();

      SideDependentList<SingleSupportStartCondition> singleSupportStartConditions = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) simulationTestHelper.findVariable(sidePrefix + "FootControlModule",
                                                                                                                      sidePrefix + "FootCurrentState");
         String prefix = robotSide.getSideNameFirstLowerCaseLetter();
         YoBoolean trustFootSwitch = ((YoBoolean) simulationTestHelper.findVariable(prefix + "_footTrustFootSwitch"));
         YoBoolean trustStateEstimatorFootSwitch = ((YoBoolean) simulationTestHelper.findVariable(prefix + "_footStateEstimatorTrustFootSwitch"));
         trustFootSwitch.set(false);
         trustStateEstimatorFootSwitch.set(false);
         YoBoolean controllerDetectedTouchdown = ((YoBoolean) simulationTestHelper.findVariable(prefix + "_footControllerDetectedTouchdown"));
         YoBoolean controllerStateEstimatorDetectedTouchdown = ((YoBoolean) simulationTestHelper.findVariable(prefix
               + "_footStateEstimatorControllerDetectedTouchdown"));
         singleSupportStartConditions.put(robotSide,
                                          new SingleSupportStartCondition(footConstraintType,
                                                                          controllerDetectedTouchdown,
                                                                          controllerStateEstimatorDetectedTouchdown));
      }

      YoDouble time = simulationTestHelper.getSimulationSession().getTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      EarlyTouchdownController earlyTouchdownController = new EarlyTouchdownController(singleSupportStartConditions, swing, time);
      earlyTouchdownController.setFractionToTriggerTouchdown(0.75);
      simulationTestHelper.getRobot().getControllerManager().addController(earlyTouchdownController);

      ThreadTools.sleep(1000);

      setupCameraSideView();

      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage footMessage = new FootstepDataListMessage();
      footMessage.setOffsetFootstepsWithExecutionError(true);
      footMessage.setOffsetFootstepsHeightWithExecutionError(true);
      ArrayList<Point3D> rootLocations = new ArrayList<>();

      PelvisCheckpointChecker controllerSpy = new PelvisCheckpointChecker(simulationTestHelper);

      for (int currentStep = 0; currentStep < numberOfSteps; currentStep++)
      {
         FootstepDataMessage footstepData = new FootstepDataMessage();
         footstepData.getLocation().set(stepLength * currentStep, side.negateIfRightSide(stepWidth / 2), 0.0);
         footstepData.setRobotSide(side.toByte());
         footMessage.getFootstepDataList().add().set(footstepData);

         side = side.getOppositeSide();
      }
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.getLocation().set(stepLength * (numberOfSteps - 1), side.negateIfRightSide(stepWidth / 2), 0.0);
      footstepData.setRobotSide(side.toByte());
      footMessage.getFootstepDataList().add().set(footstepData);

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      int steps = footMessage.getFootstepDataList().size();

      controllerSpy.setFootStepCheckPoints(rootLocations, stepLength, stepWidth);
      simulationTestHelper.simulateAndWait(1.0);
      simulationTestHelper.publishToController(footMessage);
      double simulationTime = initialTransfer + (transfer + swing) * steps + 1.0;

      assertTrue(simulationTestHelper.simulateAndWait(simulationTime));
      controllerSpy.assertCheckpointsReached();
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<FootControlModule.ConstraintType> footConstraintType;
      private final YoBoolean touchdownDetected;
      private final YoBoolean estimatorTouchdownDetection;

      public SingleSupportStartCondition(YoEnum<FootControlModule.ConstraintType> footConstraintType,
                                         YoBoolean touchdownDetected,
                                         YoBoolean estimatorTouchdownDetection)
      {
         this.footConstraintType = footConstraintType;
         this.touchdownDetected = touchdownDetected;
         this.estimatorTouchdownDetection = estimatorTouchdownDetection;
      }

      @Override
      public boolean testCondition(double time)
      {
         return footConstraintType.getEnumValue() == FootControlModule.ConstraintType.SWING;
      }

      public void triggerTouchdown()
      {
         touchdownDetected.set(true);
         estimatorTouchdownDetection.set(true);
      }
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private class EarlyTouchdownController implements Controller
   {
      private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      private final SideDependentList<SingleSupportStartCondition> startConditions;
      private final double swingDuration;

      private final YoDouble time;
      private final SideDependentList<YoDouble> timeAtSwitchStart = new SideDependentList<>();
      private final SideDependentList<YoBoolean> lastTickInSwing = new SideDependentList<>();

      private final YoDouble fractionToTriggerTouchdown = new YoDouble("FractionToTriggerTouchdown", registry);

      public EarlyTouchdownController(SideDependentList<SingleSupportStartCondition> startConditions, double swingDuration, YoDouble time)
      {
         this.startConditions = startConditions;
         this.swingDuration = swingDuration;
         this.time = time;

         for (RobotSide robotSide : RobotSide.values)
         {
            timeAtSwitchStart.put(robotSide, new YoDouble(robotSide.getCamelCaseName() + "TImeAtSwingStart", registry));
            lastTickInSwing.put(robotSide, new YoBoolean(robotSide.getCamelCaseName() + "LastTickInSwing", registry));
         }

         fractionToTriggerTouchdown.set(1.0);
      }

      public void setFractionToTriggerTouchdown(double fractionToTriggerTouchdown)
      {
         this.fractionToTriggerTouchdown.set(fractionToTriggerTouchdown);
      }

      @Override
      public void doControl()
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            boolean inSwing = startConditions.get(robotSide).testCondition(0.0);

            if (inSwing && !lastTickInSwing.get(robotSide).getBooleanValue())
            {
               timeAtSwitchStart.get(robotSide).set(time.getDoubleValue());
            }
            lastTickInSwing.get(robotSide).set(inSwing);

            double timeInState = time.getDoubleValue() - timeAtSwitchStart.get(robotSide).getDoubleValue();
            if (inSwing && timeInState > fractionToTriggerTouchdown.getDoubleValue() * swingDuration)
            {
               startConditions.get(robotSide).triggerTouchdown();
            }
         }
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }
}

package us.ihmc.avatar.fallRecovery;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.falling.FallingControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerFailedTransitionFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class SimpleFallingRecoveryTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      drcSimulationTestHelper.registerHighLevelControllerState(new FallingControllerStateFactory());
      drcSimulationTestHelper.registerControllerStateTransition(new ControllerFailedTransitionFactory(HighLevelControllerName.WALKING,
                                                                                                      HighLevelControllerName.FALLING_STATE));
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
   }

   @AfterEach
   public void tearDown()
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testFallingStateTriggered() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RigidBodyBasics chest = fullRobotModel.getChest();
      PushRobotController pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(),
                                                                        chest.getParentJoint().getName(),
                                                                        chest.getBodyFixedFrame().getTransformToParent().getTranslation());
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      double magnitude = 6.0 * TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      pushRobotController.applyForce(Axis.X, magnitude, 0.1);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertFalse(success);
      assertTrue(drcSimulationTestHelper.getCaughtException() instanceof ControllerFailureException);

      drcSimulationTestHelper.resetControllerFailure();

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);

      HighLevelControllerName currentHighLevelControlState = drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getCurrentHighLevelControlState();
      assertEquals(HighLevelControllerName.FALLING_STATE, currentHighLevelControlState);
   }
}

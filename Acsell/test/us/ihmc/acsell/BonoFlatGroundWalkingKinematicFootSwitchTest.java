package us.ihmc.acsell;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.steppr.controlParameters.BonoStateEstimatorParameters;
import us.ihmc.steppr.controlParameters.BonoWalkingControllerParameters;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(planType = BambooPlanType.InDevelopment)
public class BonoFlatGroundWalkingKinematicFootSwitchTest extends DRCFlatGroundWalkingTest
{
   private BonoRobotModel robotModel;

	@DeployableTestMethod(duration = 15.4)
	@Test(timeout = 77000)
   public void testBONOFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "BONOFlatGroundWalkingTest";
      final boolean runningOnRealRobot = false;
      
      //create a subclass of standard DRCRobot model but overrides FootSwitchType for both WalkingControl/StateEstimation parameters.
      robotModel = new BonoRobotModel(runningOnRealRobot, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new BonoWalkingControllerParameters(super.getJointMap(), runningOnRealRobot)
            {
               @Override
               public FootSwitchType getFootSwitchType()
               {
                  return FootSwitchType.KinematicBased;
               }
            };
         }

         @Override
         public StateEstimatorParameters getStateEstimatorParameters()
         {
            return new BonoStateEstimatorParameters(runningOnRealRobot, getEstimatorDT())
            {
               @Override
               public FootSwitchType getFootSwitchType()
               {
                  return FootSwitchType.KinematicBased;
               }
            };

         }
      };

      boolean doPelvisYawWarmup = false;
      setupAndTestFlatGroundSimulationTrack(robotModel, runName, doPelvisYawWarmup);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.BONO);
   }
}

package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingWithIMUDriftTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.signalCorruption.OrientationConstantAcceleratingYawDriftCorruptor;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class ValkyrieFlatGroundWalkingWithIMUDriftTest extends DRCFlatGroundWalkingWithIMUDriftTest
{
   private DRCRobotModel robotModel;

   /**
    * Need to fix the signal corruptors for the simulated sensors.
    * 
    * @throws SimulationExceededMaximumTimeException
    * @throws ControllerFailureException
    */
   @Ignore
	@DeployableTestMethod(estimatedDuration = 50.3, quarantined = true)
	@Test(timeout = 150919)
   public void testValkyrieFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      String runName = "ValkyrieFlatGroundWalkingTest";
      robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);

      DRCFlatGroundWalkingTrack track = setupFlatGroundSimulationTrack(robotModel);
      YoVariable[] allVariables = track.getSimulationConstructionSet().getAllVariablesArray();

      ArrayList<DoubleYoVariable> yawDriftAccelerationVariables = new ArrayList<>();

      for (YoVariable<?> yoVariable : allVariables)
      {
         if (yoVariable.getName().endsWith(OrientationConstantAcceleratingYawDriftCorruptor.SIMULATED_YAW_DRIFT_ACCELERATION))
            yawDriftAccelerationVariables.add((DoubleYoVariable) yoVariable);
      }

      double driftAccelerationMagnitude = 1.0;
      double sign = 1.0;

      for (int i = 0; i < yawDriftAccelerationVariables.size(); i++)
      {
         yawDriftAccelerationVariables.get(i).set(sign * driftAccelerationMagnitude);
         sign *= -driftAccelerationMagnitude;
      }
      
      simulateAndAssertGoodWalking(track, runName);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

}

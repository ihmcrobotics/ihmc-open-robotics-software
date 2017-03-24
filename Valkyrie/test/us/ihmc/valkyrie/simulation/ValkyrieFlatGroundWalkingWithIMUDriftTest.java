package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.DRCFlatGroundWalkingWithIMUDriftTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.sensorProcessing.signalCorruption.OrientationConstantAcceleratingYawDriftCorruptor;
import us.ihmc.simulationconstructionsettools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

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
	@ContinuousIntegrationTest(estimatedDuration = 50.3)
	@Test(timeout = 150919)
   public void testValkyrieFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(getSimulationTestingParameters().getShowWindows());

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

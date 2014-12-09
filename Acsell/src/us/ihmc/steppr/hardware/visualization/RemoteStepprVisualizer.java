package us.ihmc.steppr.hardware.visualization;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.steppr.hardware.StepprDashboard;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class RemoteStepprVisualizer extends SCSVisualizer
{
   public RemoteStepprVisualizer(Robot robot, int bufferSize)
   {
      super(robot, bufferSize);
   }

   @Override
   public void starting(SimulationConstructionSet scs, Robot robot, YoVariableRegistry registry)
   {
      StepprDashboard.createDashboard(scs, registry);
   }
   
   public static void main(String[] args)
   {
      System.out.println("Connecting to host " + StepprNetworkParameters.CONTROL_COMPUTER_HOST);
      BonoRobotModel robotModel = new BonoRobotModel(true, false);
      SDFRobot robot = robotModel.createSdfRobot(false);

      SCSVisualizer scsYoVariablesUpdatedListener = new RemoteStepprVisualizer(robot, 16384);

    
      
      YoVariableClient client = new YoVariableClient(StepprNetworkParameters.CONTROL_COMPUTER_HOST, scsYoVariablesUpdatedListener,
            "remote", false);
      client.start();

   }
}

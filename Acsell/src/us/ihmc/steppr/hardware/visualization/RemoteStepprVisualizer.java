package us.ihmc.steppr.hardware.visualization;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;
import us.ihmc.steppr.hardware.StepprDashboard;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;

import us.ihmc.simulationconstructionset.Robot;

public class RemoteStepprVisualizer extends SCSVisualizer
{
   public RemoteStepprVisualizer(Robot robot, int bufferSize)
   {
      super(robot, bufferSize);
   }

   @Override
   public void start()
   {
      StepprDashboard.createDashboard(scs, registry);
      
      super.start();
   }
   
   public static void main(String[] args)
   {
      System.out.println("Connecting to host " + StepprNetworkParameters.CONTROL_COMPUTER_HOST);
      BonoRobotModel robotModel = new BonoRobotModel(true, false);
      SDFRobot robot = robotModel.createSdfRobot(false);

      SCSVisualizer scsYoVariablesUpdatedListener = new RemoteStepprVisualizer(robot, 16384);

    
      
      YoVariableClient client = new YoVariableClient(StepprNetworkParameters.CONTROL_COMPUTER_HOST, StepprNetworkParameters.VARIABLE_SERVER_PORT,
            scsYoVariablesUpdatedListener, "remote", false);
      client.start();

   }
}

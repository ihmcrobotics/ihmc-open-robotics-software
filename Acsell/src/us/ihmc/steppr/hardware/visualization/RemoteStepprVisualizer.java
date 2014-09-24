package us.ihmc.steppr.hardware.visualization;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSYoVariablesUpdatedListener;
import us.ihmc.steppr.hardware.StepprDashboard;
import us.ihmc.steppr.hardware.configuration.StepprNetworkParameters;

import com.yobotics.simulationconstructionset.Robot;

public class RemoteStepprVisualizer extends SCSYoVariablesUpdatedListener
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

      SCSYoVariablesUpdatedListener scsYoVariablesUpdatedListener = new RemoteStepprVisualizer(robot, 16384);

    
      
      YoVariableClient client = new YoVariableClient(StepprNetworkParameters.CONTROL_COMPUTER_HOST, StepprNetworkParameters.VARIABLE_SERVER_PORT,
            scsYoVariablesUpdatedListener, "remote", false);
      client.start();

   }
}

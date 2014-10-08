package us.ihmc.humanoidBehaviors;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;

import com.yobotics.simulationconstructionset.Robot;

public class BehaviorVisualizer extends SCSVisualizer
{
   private final boolean showOverheadView = true;
   
   public BehaviorVisualizer(String host, int bufferSize, Robot robot)
   {
      super(robot, bufferSize);
      
      YoVariableClient client = new YoVariableClient(host, IHMCHumanoidBehaviorManager.BEHAVIOR_YO_VARIABLE_SERVER_PORT, this, "behavior", showOverheadView);
      client.start();
   }
   
   public static void main(String[] arg)
   {
      new BehaviorVisualizer("localhost",16300,new Robot("theInvisibleRobot"));
   }
}

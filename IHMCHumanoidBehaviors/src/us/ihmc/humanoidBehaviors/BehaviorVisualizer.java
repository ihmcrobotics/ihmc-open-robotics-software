package us.ihmc.humanoidBehaviors;

import us.ihmc.communication.util.NetworkConfigParameters;
import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;

import us.ihmc.simulationconstructionset.Robot;

public abstract class BehaviorVisualizer extends SCSVisualizer
{
   private final boolean showOverheadView = false;
   
   public BehaviorVisualizer(String host, int bufferSize, Robot robot)
   {
      super(robot, bufferSize, true, false);
      
      YoVariableClient client = new YoVariableClient(host, NetworkConfigParameters.BEHAVIOR_YO_VARIABLE_SERVER_PORT, this, "behavior", showOverheadView);
      client.start();
   }
}

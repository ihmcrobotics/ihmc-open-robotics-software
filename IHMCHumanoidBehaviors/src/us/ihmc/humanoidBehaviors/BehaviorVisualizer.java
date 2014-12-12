package us.ihmc.humanoidBehaviors;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;

public class BehaviorVisualizer extends SCSVisualizer
{
   private final boolean showOverheadView = false;
   
   public BehaviorVisualizer(String host, int bufferSize)
   {
      super(bufferSize, true, false);
      
      YoVariableClient client = new YoVariableClient(host, this, "behavior", showOverheadView);
      client.start();
   }
}

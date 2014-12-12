package us.ihmc.humanoidBehaviors;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;

public class BehaviorVisualizer extends SCSVisualizer
{
   private final boolean showOverheadView = false;
   
   private final static boolean onRealRobot = false;
   
   public BehaviorVisualizer(String host, int bufferSize)
   {
      super(bufferSize, true, false);
      
      YoVariableClient client = new YoVariableClient(host, this, "behavior", showOverheadView);
      client.start();
   }
   
   public static void main(String[] args)
   {
      if (!onRealRobot)
         new BehaviorVisualizer("127.0.0.1", 1024*32);
      else
         new BehaviorVisualizer("10.66.171.20", 1024*32);
         
   }
}

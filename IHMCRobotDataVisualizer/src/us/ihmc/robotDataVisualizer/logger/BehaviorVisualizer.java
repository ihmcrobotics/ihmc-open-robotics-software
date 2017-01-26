package us.ihmc.robotDataVisualizer.logger;

import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;

public class BehaviorVisualizer extends SCSVisualizer
{
   public BehaviorVisualizer(int bufferSize)
   {
      super(bufferSize, true, false);
      setShowOverheadView(false);

      YoVariableClient client = new YoVariableClient(this, "behavior");
      client.start();
   }

   public static void main(String[] args)
   {
      JSAP jsap = new JSAP();
      Switch runningOnRealRobotSwitch = new Switch("runningOnRealRobot").setLongFlag("realRobot");

      try
      {
         jsap.registerParameter(runningOnRealRobotSwitch);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            new BehaviorVisualizer(1024 * 32);
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }
   }
}

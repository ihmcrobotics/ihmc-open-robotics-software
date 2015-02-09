package us.ihmc.humanoidBehaviors;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.visualizer.SCSVisualizer;

import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

public class BehaviorVisualizer extends SCSVisualizer
{
   private static final String LOCAL_IP = "127.0.0.1";
   private static final String CONTROLLER_IP = "192.168.130.112";

   private final boolean showOverheadView = false;

   public BehaviorVisualizer(String host, int bufferSize)
   {
      super(bufferSize, true, false);

      YoVariableClient client = new YoVariableClient(host, this, "behavior", showOverheadView);
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
            boolean runningOnRealRobot = config.getBoolean(runningOnRealRobotSwitch.getID());
            String ipAddressToUse = runningOnRealRobot ? CONTROLLER_IP : LOCAL_IP;
            new BehaviorVisualizer(ipAddressToUse, 1024 * 32);
         }
      }
      catch (JSAPException e)
      {
         e.printStackTrace();
      }
   }
}

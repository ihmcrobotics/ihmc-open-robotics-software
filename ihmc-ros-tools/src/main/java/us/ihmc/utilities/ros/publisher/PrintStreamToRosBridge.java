package us.ihmc.utilities.ros.publisher;

import java.io.PrintStream;
import java.util.concurrent.TimeUnit;

import us.ihmc.tools.io.TimerBasedOutputStream;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTopicEchoer;

public class PrintStreamToRosBridge extends PrintStream
{
   private static final int TIME_PERIOD = 1;
   private static final TimeUnit TIME_UNIT = TimeUnit.MILLISECONDS;
   
   private TimerBasedOutputStream tbos = (TimerBasedOutputStream) out;
   private RosLogPublisher publisher;
   private RosTopicEchoer<rosgraph_msgs.Log> echoPublisher;
   private Thread currentThread = null;
   
   public PrintStreamToRosBridge(RosMainNode rosMainNode, String rosNamespace)
   {
      super(new TimerBasedOutputStream(TIME_PERIOD, TIME_UNIT), true);
      
      publisher = new RosLogPublisher(rosMainNode, true);
      rosMainNode.attachPublisher(rosNamespace + "/ihmc_err", publisher);
      
      echoPublisher = new RosTopicEchoer<>(rosMainNode, rosgraph_msgs.Log._TYPE, rosNamespace + "/ihmc_err", "/rosout");
   }
   
   public PrintStreamToRosBridge()
   {
      super(new TimerBasedOutputStream(TIME_PERIOD, TIME_UNIT), true);
   }
   
   public void start()
   {
      if(currentThread == null)
      {
         currentThread = new Thread(new OutputStreamMonitor());
         currentThread.start();
      }
   }
   
   private class OutputStreamMonitor implements Runnable
   {
      @Override
      public void run()
      {
         while(true)
         {
            if(tbos.hasNewOutput())
            {
               publisher.publish((byte) 8, tbos.getString());
            }
         }
      }
   }
}

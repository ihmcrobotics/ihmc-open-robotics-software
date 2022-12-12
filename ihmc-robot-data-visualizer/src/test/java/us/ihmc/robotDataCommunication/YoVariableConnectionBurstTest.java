package us.ihmc.robotDataCommunication;

import static us.ihmc.robotics.Assert.*;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.util.PeriodicNonRealtimeThreadSchedulerFactory;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

public class YoVariableConnectionBurstTest
{
   enum TestEnum
   {
      A, B, C, D
   }
   
   private final YoRegistry registry = new YoRegistry("tester");
   private final YoLong seq_id = new YoLong("seq_id", registry);
   private final YoLong sleep = new YoLong("sleep", registry);
   private final YoEnum<TestEnum> var3 = new YoEnum<TestEnum>("var3", "", registry, TestEnum.class, true);
   
   

   
   
//   @Test
   public void TestYoVariableConnectionBurst()
   {
      

	  //start server
      final YoVariableServer server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadSchedulerFactory(), null, new DataServerSettings(false), 0.001);
      server.setMainRegistry(registry, null);
      server.start();

      
      //start client
      int bufferSize=256;
      SCSVisualizer scsYoVariablesUpdatedListener = new SCSVisualizer(bufferSize, false);      
      scsYoVariablesUpdatedListener.setDisplayOneInNPackets(1);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);

      final YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener);
      client.startWithHostSelector();
      
      
      ThreadTools.sleep(1000); //ensure connections

      
      //start a producer/consumer test with frequent burst send
      seq_id.set(0L);

      long timestamp = 0;
      int i = 0;
      TestEnum[] values = { TestEnum.A, TestEnum.B, TestEnum.C, TestEnum.D };
      
      for(int iter=0;iter<100;iter++)
      {
         seq_id.increment();
         
         if(++i >= values.length)
         {
            i = 0;
         }
         var3.set(values[i]);
         
         server.update(++timestamp);
         
         
         if (iter < 50)
        	 sleep.set(5);
         else
         {
	         if(iter % 10 !=0)
	        	 sleep.set(5);
	         else
	        	 sleep.set(0);
         }
    	 ThreadTools.sleep(sleep.getLongValue());
      }
      

      ThreadTools.sleep(1000);
      
      
      //make sure last nCheck seq_ids are consecutive.
      final int nCheck=20;
      YoBuffer buffer=scsYoVariablesUpdatedListener.getDataBuffer();
      YoLong seq =  (YoLong)buffer.findVariable("seq_id");
      buffer.setLockIndex(true);
      long lastSeq = seq.getLongValue();
      int lastIndex = buffer.getCurrentIndex();
      
      for(int j=0;j<nCheck;j++)
      {
    	  buffer.setCurrentIndex(lastIndex-j);
    	  assertEquals(seq.getLongValue() , (lastSeq-j));
      }
      
      
      scsYoVariablesUpdatedListener.closeAndDispose();
   }
   
   
   public static void main(String[] arg)
   {
	   new YoVariableConnectionBurstTest();
   }
}

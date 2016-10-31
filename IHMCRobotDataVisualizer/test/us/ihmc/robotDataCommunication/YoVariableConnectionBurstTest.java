package us.ihmc.robotDataCommunication;

import static org.junit.Assert.assertEquals;

import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotDataVisualizer.visualizer.SCSVisualizer;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

public class YoVariableConnectionBurstTest
{
   enum TestEnum
   {
      A, B, C, D
   }
   
   private final YoVariableRegistry registry = new YoVariableRegistry("tester");
   private final LongYoVariable seq_id = new LongYoVariable("seq_id", registry);
   private final LongYoVariable sleep = new LongYoVariable("sleep", registry);
   private final EnumYoVariable<TestEnum> var3 = new EnumYoVariable<TestEnum>("var3", "", registry, TestEnum.class, true);
   
   

   
   
//   @Test(timeout=300000)
   public void TestYoVariableConnectionBurst()
   {
      

	  //start server
      final YoVariableServer server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler("YoVariableConnectionBurstTest"), null, LogSettings.SIMULATION, 0.001);
      server.setMainRegistry(registry, null, null);
      server.start();

      
      //start client
      int bufferSize=256;
      SCSVisualizer scsYoVariablesUpdatedListener = new SCSVisualizer(bufferSize, false);      
      scsYoVariablesUpdatedListener.setDisplayOneInNPackets(1);
      scsYoVariablesUpdatedListener.setShowOverheadView(false);

      final YoVariableClient client = new YoVariableClient(scsYoVariablesUpdatedListener, "");
      client.start();
      
      
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
      DataBuffer buffer=scsYoVariablesUpdatedListener.getDataBuffer();
      LongYoVariable seq =  (LongYoVariable)buffer.getVariable("seq_id");
      buffer.setSafeToChangeIndex(true);
      long lastSeq = seq.getLongValue();
      int lastIndex = buffer.getIndex();
      
      for(int j=0;j<nCheck;j++)
      {
    	  buffer.setIndex(lastIndex-j);
    	  assertEquals(seq.getLongValue() , (lastSeq-j));
      }
      
      
      scsYoVariablesUpdatedListener.closeAndDispose();
   }
   
   
   public static void main(String[] arg)
   {
	   new YoVariableConnectionBurstTest();
   }
}

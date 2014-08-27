package us.ihmc.robotDataCommunication;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.robotDataCommunication.visualizer.SCSYoVariablesUpdatedListener;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.DataBuffer;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.LongYoVariable;

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
   
   

   
   
   @Test
   public void TestYoVariableConnectionBurst()
   {
      

	  //start server
      final YoVariableServer server = new YoVariableServer(1234, 0.001);
      server.setMainRegistry(registry, null, null);
      server.start();

      
      //start client
      int bufferSize=256;
      SCSYoVariablesUpdatedListener scsYoVariablesUpdatedListener = new SCSYoVariablesUpdatedListener(bufferSize, false);      
      scsYoVariablesUpdatedListener.setDisplayOneInNPackets(1);
      final YoVariableClient client = new YoVariableClient("localhost", 1234, scsYoVariablesUpdatedListener, "", false);
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
         
         server.update(++timestamp, 0);
         
         
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

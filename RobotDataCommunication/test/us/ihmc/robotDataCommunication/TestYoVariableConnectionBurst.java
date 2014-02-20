package us.ihmc.robotDataCommunication;

import us.ihmc.robotDataCommunication.visualizer.SCSYoVariablesUpdatedListener;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.LongYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


public class TestYoVariableConnectionBurst
{
   enum TestEnum
   {
      A, B, C, D
   }
   
   private final YoVariableRegistry registry = new YoVariableRegistry("tester");
   private final LongYoVariable seq_id = new LongYoVariable("seq_id", registry);
   private final LongYoVariable sleep = new LongYoVariable("sleep", registry);
   private final EnumYoVariable<TestEnum> var3 = new EnumYoVariable<TestEnum>("var3", "", registry, TestEnum.class, true);
   
   
   private final YoVariableServer server = new YoVariableServer(registry, 1234, 0.001);
   private final YoVariableClient client;
   
   
   public TestYoVariableConnectionBurst()
   {
      int bufferSize=256;
      SCSYoVariablesUpdatedListener scsYoVariablesUpdatedListener = new SCSYoVariablesUpdatedListener(bufferSize);      

      server.start();
      boolean showOverheadView = false;
      client = new YoVariableClient("localhost", 1234, scsYoVariablesUpdatedListener, "", showOverheadView);
      client.start();

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
         
         
         if (iter > 50)
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

      client.close();      
   }
   
   public static void main(String[] args)
   {
      new TestYoVariableConnectionBurst();
   }
}

package us.ihmc.robotDataCommunication;

import us.ihmc.robotDataCommunication.YoVariableClient;
import us.ihmc.robotDataCommunication.YoVariableServer;
import us.ihmc.robotDataCommunication.visualizer.SCSYoVariablesUpdatedListener;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class TestYoVariableConnection
{
   enum TestEnum
   {
      A, B, C, D
   }
   
   private final YoVariableRegistry registry = new YoVariableRegistry("tester");
   private final DoubleYoVariable var1 = new DoubleYoVariable("var1", registry);
   private final DoubleYoVariable var2 = new DoubleYoVariable("var2", registry);
   private final IntegerYoVariable var4 = new IntegerYoVariable("var4", registry);
   private final EnumYoVariable<TestEnum> var3 = new EnumYoVariable<TestEnum>("var3", "", registry, TestEnum.class, true);
   
   private final YoVariableServer server = new YoVariableServer(registry, 1234, 0.001);
   private final YoVariableClient client;
   
   
   public TestYoVariableConnection()
   {
      int bufferSize=8192;
      SCSYoVariablesUpdatedListener scsYoVariablesUpdatedListener = new SCSYoVariablesUpdatedListener(bufferSize);

      server.start();
      client = new YoVariableClient("localhost", 1234, scsYoVariablesUpdatedListener, "");
      client.start();
      var4.set(5000);
      
      long timestamp = 0;
      int i = 0;
      TestEnum[] values = { TestEnum.A, TestEnum.B, TestEnum.C, TestEnum.D };
      while(true)
      {
         var1.add(1.0);
         var2.sub(1.0);
         var4.subtract(1);
         
         if(++i >= values.length)
         {
            i = 0;
         }
         var3.set(values[i]);
         
         server.update(++timestamp);
         ThreadTools.sleep(1);
      }
   }
   
   public static void main(String[] args)
   {
      new TestYoVariableConnection();
   }
}

package us.ihmc.robotDataCommunication;

import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;


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
   
   private final IntegerYoVariable echoIn = new IntegerYoVariable("echoIn", registry);
   private final IntegerYoVariable echoOut = new IntegerYoVariable("echoOut", registry);
   
   private final YoVariableServer server = new YoVariableServer(getClass(), new PeriodicNonRealtimeThreadScheduler("TestYoVariableConnection"), null, LogSettings.SIMULATION, 0.001);
   
   
   private volatile long timestamp = 0;
   
   public TestYoVariableConnection()
   {
      server.setMainRegistry(registry, null, null);
      new ThreadTester(server).start();
      server.start();
      var4.set(5000);
      
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
         
         echoOut.set(echoIn.getIntegerValue());
         
         server.update(++timestamp);
         ThreadTools.sleep(1);
      }
   }
   
   private class ThreadTester extends Thread
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("Thread");
      private final DoubleYoVariable A = new DoubleYoVariable("A", registry);
      private final DoubleYoVariable B = new DoubleYoVariable("B", registry);
      private final DoubleYoVariable C = new DoubleYoVariable("C", registry);
      
      
      private final EnumYoVariable<TestEnum> echoThreadIn = new EnumYoVariable<TestEnum>("echoThreadIn", registry, TestEnum.class, false);
      private final EnumYoVariable<TestEnum> echoThreadOut = new EnumYoVariable<TestEnum>("echoThreadOut", registry, TestEnum.class, false);

      
      public ThreadTester(YoVariableServer server)
      {
         server.addRegistry(registry, null);
      }
      
      public void run()
      {
         while(true)
         {
            A.set(A.getDoubleValue() + 0.5);
            B.set(B.getDoubleValue() - 0.5);
            C.set(C.getDoubleValue() * 2.0);
            
            echoThreadOut.set(echoThreadIn.getEnumValue());
            
            server.update(timestamp, registry);
            
            ThreadTools.sleep(10);
         }
      }
      
      
   }
   
   public static void main(String[] args)
   {
      new TestYoVariableConnection();
   }
}

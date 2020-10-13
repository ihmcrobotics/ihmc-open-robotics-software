package us.ihmc.tools.processManagement;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;

import java.util.ArrayList;

public class JavaProcessManagerTest
{
   @Test
   public void testJavaProcessManager()
   {
      main(null);
   }

   public static void main(String[] args)
   {
      JavaProcessManager manager = new JavaProcessManager();
      manager.runOrRegister("HelloWorld", () -> System.out.println("Hello World"));

      if (manager.isSpawnerProcess())
      {
         ArrayList<Process> processes = manager.spawnProcesses(JavaProcessManagerTest.class, args);

         ThreadTools.sleepSeconds(2.0);

         for (Process process : processes)
         {
            process.destroy();
         }
      }
   }
}

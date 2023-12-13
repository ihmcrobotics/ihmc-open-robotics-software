package us.ihmc.rdx.ui.processes;

import us.ihmc.tools.processManagement.JavaProcessSpawner;

public class RestartableJavaProcess extends RestartableProcess
{
   private final JavaProcessSpawner spawner;
   private final Class<?> mainClass;
   private final String[] javaArgs;
   private Process process;
   private int destroyCount = 0;

   public RestartableJavaProcess(Class<?> mainClass, String[] javaArgs)
   {
      this.mainClass = mainClass;
      this.javaArgs = javaArgs;
      spawner = new JavaProcessSpawner(true, true);
   }

   @Override
   protected void startInternal()
   {
      process = spawner.spawn(mainClass, javaArgs, null);
      destroyCount = 0;
   }

   @Override
   protected void stopInternal()
   {
      if (destroyCount > 1)
      {
         process.destroyForcibly();
      }
      else
      {
         process.destroy();
      }

      destroyCount++;
   }

   @Override
   public String getName()
   {
      return mainClass.getSimpleName();
   }
}

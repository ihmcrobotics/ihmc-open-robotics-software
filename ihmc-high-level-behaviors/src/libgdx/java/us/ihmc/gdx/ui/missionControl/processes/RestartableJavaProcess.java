package us.ihmc.gdx.ui.missionControl.processes;

import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public class RestartableJavaProcess extends RestartableMissionControlProcess
{
   private final JavaProcessSpawner spawner;
   private final Class<?> mainClass;
   private Process process;
   private int destroyCount = 0;

   public RestartableJavaProcess(Class<?> mainClass)
   {
      this.mainClass = mainClass;
      spawner = new JavaProcessSpawner(true, true);
   }

   @Override
   protected void startInternal()
   {
      process = spawner.spawn(mainClass);
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

package us.ihmc.avatar.networkProcessor;

import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.tools.thread.CloseableAndDisposable;

public class ClosableAndDisposableSpawnedProcess implements CloseableAndDisposable
{
   private final JavaProcessSpawner javaProcessSpawner;
   private final Process process;

   public ClosableAndDisposableSpawnedProcess(Class clazz)
   {
      javaProcessSpawner = new JavaProcessSpawner(true, true);
      process = javaProcessSpawner.spawn(clazz);
   }

   @Override
   public void closeAndDispose()
   {
      javaProcessSpawner.kill(process);
   }
}

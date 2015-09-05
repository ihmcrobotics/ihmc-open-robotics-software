package us.ihmc.tools.thread;

import java.util.ArrayList;

/**
 * CloseableAndDisposableRegistry is a registry for CloseableAndDisposable objects.
 * These are things like threads that need to shutdown gently.
 * Since Java deprecated all the kill methods on Threads for safety, it is difficult to tell a 
 * Thread to shut down. One solution used here is to make things that need to be closed and disposed
 * a CloseableAndDisposable. Then use a CloseableAndDisposableRegistry to register such objects.
 * At the highest level then, when things need to get shutdown, one can just call the closeAndDispose() method
 * on the CloseableAndDisposableRegistry.
 */
public class CloseableAndDisposableRegistry
{
   private ArrayList<CloseableAndDisposable> closeableAndDisposables = new ArrayList<CloseableAndDisposable>();
   
   public CloseableAndDisposableRegistry()
   {
   }
   
   public void registerCloseable(CloseableAndDisposable closeable)
   {
      this.closeableAndDisposables.add(closeable);
   }
   
   public void closeAndDispose()
   {
      for (CloseableAndDisposable closeable : closeableAndDisposables)
      {
         closeable.closeAndDispose();
      }
      
      closeableAndDisposables.clear();
      closeableAndDisposables = null;
   }
}

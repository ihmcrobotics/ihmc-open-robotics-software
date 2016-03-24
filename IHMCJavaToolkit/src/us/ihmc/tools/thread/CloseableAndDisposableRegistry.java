package us.ihmc.tools.thread;

import java.util.ArrayList;
import java.util.List;

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
   private List<CloseableAndDisposable> closeableAndDisposables = new ArrayList<>();
   private List<CloseableAndDisposableRegistry> children = new ArrayList<>();

   public CloseableAndDisposableRegistry()
   {
   }

   public void registerCloseableAndDisposable(CloseableAndDisposable closeable)
   {
      this.closeableAndDisposables.add(closeable);
   }

   public void registerCloseablesAndDisposables(List<CloseableAndDisposable> closeables)
   {
      for (int i = 0; i < closeables.size(); i++)
         registerCloseableAndDisposable(closeables.get(i));
   }

   public void registerChild(CloseableAndDisposableRegistry other)
   {
      children.add(other);
   }

   public void closeAndDispose()
   {
      for (CloseableAndDisposable closeableAndDisposable : closeableAndDisposables)
      {
         closeableAndDisposable.closeAndDispose();
      }

      closeableAndDisposables.clear();
      closeableAndDisposables = null;

      for (CloseableAndDisposableRegistry closeableAndDisposableRegistry : children)
      {
         closeableAndDisposableRegistry.closeAndDispose();
      }

      children.clear();
      children = null;
   }
}

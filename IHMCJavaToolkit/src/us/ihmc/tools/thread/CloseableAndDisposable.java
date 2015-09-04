package us.ihmc.tools.thread;

/**
 * CloseableAndDisposable is an interface for things like threads that need to shutdown gently.
 * Since Java deprecated all the kill methods on Threads for safety, it is difficult to tell a 
 * Thread to shut down. One solution used here is to make things that need to be closed and disposed
 * a CloseableAndDisposable. Then use a CloseableAndDisposableRegistry to register such objects.
 * At the highest level then, when things need to get shutdown, one can just call the closeAndDispose() method
 * on the CloseableAndDisposableRegistry.
 */
public interface CloseableAndDisposable
{
   public abstract void closeAndDispose();
}

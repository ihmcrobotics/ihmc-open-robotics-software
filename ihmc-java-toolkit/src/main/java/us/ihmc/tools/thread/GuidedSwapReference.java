package us.ihmc.tools.thread;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * <p>
 * Manages two data instances shared by two threads. This often can be used to
 * double performance by doing reading and writing at the same time, if both reading
 * and writing need some time to access the data. Since there's two data instances,
 * the data references can just be exchanged instead of copying anything.
 * </p><p>
 * "Guided" refers to this class being "on rails" compared to {@link SwapReference}.
 * </p><p>
 * It is expected that only two threads be operating on this class throughout its
 * existence. Be careful to only use a single thread executor to call the low
 * priority access method.
 * </p><p>
 * The primary consideration for which thread is "low priority" vs. "high priority"
 * is on which thread is acceptable to block in the event that the other is
 * currently accessing the other data.
 * </p><p>
 * This class probably isn't bulletproof, but works under most conditions.
 * </p><p>
 * The Java synchronized statement is allocation free. Fun reading:
 * <ul>
 *    <li>https://wiki.openjdk.org/display/HotSpot/Synchronization</li>
 *    <li>https://github.com/openjdk/jdk/blob/master/src/hotspot/share/runtime/synchronizer.cpp</li>
 * </ul>
 * </p><p>
 * Also evaluate IHMC Realtime's ConcurrentCopier for your use case.
 * </p>
 */
public class GuidedSwapReference<T>
{
   private final T a;
   private final T b;
   private T forLowPriorityThread;
   private T forHighPriorityThread;
   private final Consumer<T> accessOnLowPriorityThread;
   private final Consumer<T> accessOnHighPriorityThread;

   /**
    * We are preallocating the accessors because if you are using this class, you
    * care about performance and we want to avoid allocating lots of lambdas.
    * This is also usually more convenient and readable in user code anyway.
    */
   public GuidedSwapReference(Supplier<T> supplier, Consumer<T> accessOnLowPriorityThread, Consumer<T> accessOnHighPriorityThread)
   {
      this.a = supplier.get();
      this.b = supplier.get();
      forLowPriorityThread = a;
      forHighPriorityThread = b;
      this.accessOnLowPriorityThread = accessOnLowPriorityThread;
      this.accessOnHighPriorityThread = accessOnHighPriorityThread;
   }

   /**
    * If desired, call once at the beginning to initialize the data.
    */
   public void initializeBoth(Consumer<T> consumer)
   {
      consumer.accept(a);
      consumer.accept(b);
   }

   /**
    * This thread performs the following steps in order:
    * <ol>
    *    <li>Immediately performs the low priority access on the data.</li>
    *    <li>If the high priority thread is accessing it's data, we block until that completes.</li>
    *    <li>Does the swap operation atomically, which exchanges the two data instances
    *    so the other thread can access it.</li>
    * </ol>
    */
   public void accessOnLowPriorityThread()
   {
      accessOnLowPriorityThread.accept(forLowPriorityThread);

      // We're done, let's do the swap operation.
      // This will block and wait for the high priority thread to complete its access
      // operation if necessary.
      synchronized (this)
      {
         T wasHighPriorityThread = forHighPriorityThread;
         forHighPriorityThread = forLowPriorityThread;
         forLowPriorityThread = wasHighPriorityThread;
      }
   }

   /**
    * Call this from the higher priority of the two threads. This
    * calls the cooresponding Consumer given in the constructor.
    * This method will not block.
    *
    * High priority. Get in, get out.
    * Since the other synchronized block is just over the swap operation,
    * this will effectively not block.
    */
   public synchronized void accessOnHighPriorityThread()
   {
      accessOnHighPriorityThread.accept(forHighPriorityThread);
   }
}

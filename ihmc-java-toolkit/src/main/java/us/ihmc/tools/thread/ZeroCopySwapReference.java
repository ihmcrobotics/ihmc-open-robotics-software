package us.ihmc.tools.thread;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Manages two data instances shared by two threads. This often can be used to
 * double performance by doing reading and writing at the same time, if both reading
 * and writing need some time to access the data. Since there's two data instances,
 * the data references can just be exchanged instead of copying anything.
 *
 * It is expected that only two threads be operating on this class throughout its
 * existence. Be careful to only use a single thread executor to call the low
 * priority access method.
 *
 * This class probably isn't bulletproof, but works under most conditions.
 *
 * Also evaluate IHMC Realtime's ConcurrentCopier for your use case.
 */
public class ZeroCopySwapReference<T>
{
   private final T a;
   private final T b;
   private T forLowPriorityThread;
   private T forHighPriorityThread;
   private final StatelessNotification notification = new StatelessNotification();
   private volatile boolean hightPriorityThreadIsAccessing = false;

   // Optional fields
   private Consumer<T> accessOnLowPriorityThread;
   private Consumer<T> accessOnHighPriorityThread;

   public ZeroCopySwapReference(Supplier<T> supplier, Consumer<T> accessOnLowPriorityThread, Consumer<T> accessOnHighPriorityThread)
   {
      this(supplier);
      this.accessOnLowPriorityThread = accessOnLowPriorityThread;
      this.accessOnHighPriorityThread = accessOnHighPriorityThread;
   }

   /** @deprecated This should really be phased out in favor of predefining good named method references. */
   public ZeroCopySwapReference(Supplier<T> supplier)
   {
      this(supplier.get(), supplier.get());
   }

   public ZeroCopySwapReference(T a, T b)
   {
      this.a = a;
      this.b = b;
      forLowPriorityThread = a;
      forHighPriorityThread = b;
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
    * Call if you initialized this class with consumers.
    */
   public void accessOnLowPriorityThread()
   {
      accessOnLowPriorityThread(accessOnLowPriorityThread);
   }

   /**
    * This thread does all the necessary waiting.
    * It's got access to its data straight away, that it got access to
    * last time it handed off to thread two.
    *
    * If thread two is accessing, we wait for it to be done and give it access to
    * the new data.
    */
   public void accessOnLowPriorityThread(Consumer<T> consumer)
   {
      consumer.accept(forLowPriorityThread);

      // waits for two to be done if it's reading
      if (hightPriorityThreadIsAccessing)
         notification.blockingWait();

      // we're done, let's swap access now
      forHighPriorityThread = forLowPriorityThread; // give high priority thread access to the swap right away
      forLowPriorityThread = a == forHighPriorityThread ? b : a;
   }

   /**
    * Call if you initialized this class with consumers.
    */
   public void accessOnHighPriorityThread()
   {
      accessOnHighPriorityThread(accessOnHighPriorityThread);
   }

   /**
    * Thread two is high priority reading and shouldn't be getting blocked.
    * It just says when it's accessing and notifies when it's done.
    */
   public void accessOnHighPriorityThread(Consumer<T> consumer)
   {
      // high priority get in, get out
      hightPriorityThreadIsAccessing = true;

      consumer.accept(forHighPriorityThread);

      hightPriorityThreadIsAccessing = false;
      notification.notifyOtherThread();
   }
}

package us.ihmc.tools.thread;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This class probably isn't bulletproof, but works under most conditions.
 */
public class ZeroCopySwapReference<T>
{
   private final T a;
   private final T b;
   private T forLowPriorityThread;
   private T forHighPriorityThread;
   private final StatelessNotification notification = new StatelessNotification();
   private volatile boolean hightPriorityThreadIsAccessing = false;

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

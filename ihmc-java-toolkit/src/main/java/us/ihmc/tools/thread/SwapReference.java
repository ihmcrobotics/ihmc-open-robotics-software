package us.ihmc.tools.thread;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Use for threading where one thread writes from A while another reads from B,
 * then swap and vice versa.
 *
 * Also called a double buffer, but didn't want to confuse with the data type.
 *
 * Typical usage is to operations on one instance, using a synchronized block
 * using this object to synchronize over. The swap method in this class is
 * also synchronized over this object, so it will be atomic. You may access
 * the other instance without a synchronized block freely.
 */
public class SwapReference<T>
{
   private final T a;
   private final T b;
   private T forThreadOne;
   private T forThreadTwo;

   public SwapReference(Supplier<T> supplier)
   {
      this(supplier.get(), supplier.get());
   }

   public SwapReference(T a, T b)
   {
      this.a = a;
      this.b = b;
      forThreadOne = a;
      forThreadTwo = b;
   }

   public void initializeBoth(Consumer<T> consumer)
   {
      consumer.accept(a);
      consumer.accept(b);
   }

   public T getA()
   {
      return a;
   }

   public T getB()
   {
      return b;
   }

   public T getForThreadOne()
   {
      return forThreadOne;
   }

   public T getForThreadTwo()
   {
      return forThreadTwo;
   }

   public boolean isThreadOneA()
   {
      return forThreadOne == a;
   }

   /**
    * Performs the swap operation atomically.
    */
   public synchronized void swap()
   {
      T temp = forThreadOne;
      forThreadOne = forThreadTwo;
      forThreadTwo = temp;
   }
}

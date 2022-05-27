package us.ihmc.tools.thread;

import java.util.function.Supplier;

/**
 * Use for threading where one thread writes from A while another reads from B,
 * then swap and vice versa.
 *
 * Also called a double buffer, but didn't want to confuse with the data type.
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

   public void swap()
   {
      T temp = forThreadOne;
      forThreadOne = forThreadTwo;
      forThreadTwo = temp;
   }
}

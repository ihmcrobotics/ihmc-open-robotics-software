package us.ihmc.tools;

import sun.misc.Unsafe;

import java.lang.reflect.Field;

/**
 * These things were copied in from various things found on the internet.
 * Probably doesn't work.
 */
public class UnsafeTools
{
   private static Unsafe unsafe;

   static
   {
      try
      {
         Field field = Unsafe.class.getDeclaredField("theUnsafe");
         field.setAccessible(true);
         unsafe = (Unsafe) field.get(null);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static void printAddresses(String label, Object... objects)
   {
      boolean is64bit = true;

      long last = 0;
      int offset = unsafe.arrayBaseOffset(objects.getClass());
      int scale = unsafe.arrayIndexScale(objects.getClass());
      switch (scale)
      {
         case 4:
            long factor = is64bit ? 8 : 1;
            final long i1 = (unsafe.getInt(objects, offset) & 0xFFFFFFFFL) * factor;
            System.out.print(Long.toHexString(i1));
            last = i1;
            for (int i = 1; i < objects.length; i++)
            {
               final long i2 = (unsafe.getInt(objects, offset + i * 4) & 0xFFFFFFFFL) * factor;
               if (i2 > last)
                  System.out.print(", +" + Long.toHexString(i2 - last));
               else
                  System.out.print(", -" + Long.toHexString(last - i2));
               last = i2;
            }
            break;
         case 8:
            throw new AssertionError("Not supported");
      }
      System.out.println();
   }

   public static long addressOf(Object o) throws Exception
   {
      Object[] array = new Object[] {o};

      long baseOffset = unsafe.arrayBaseOffset(Object[].class);
      int addressSize = unsafe.addressSize();
      long objectAddress;
      switch (addressSize)
      {
         case 4:
            objectAddress = unsafe.getInt(array, baseOffset);
            break;
         case 8:
            objectAddress = unsafe.getLong(array, baseOffset);
            break;
         default:
            throw new Error("unsupported address size: " + addressSize);
      }

      return (objectAddress);
   }

   public static void main(String... args) throws Exception
   {
      Object mine = "Hi there".toCharArray();
      long address = addressOf(mine);
      System.out.println("Address: " + address);

      // Verify address works - should see the characters in the array in the output
      printBytes(address, 27);
   }

   public static void printBytes(long objectAddress, int num)
   {
      for (long i = 0; i < num; i++)
      {
         int cur = unsafe.getByte(objectAddress + i);
         System.out.print((char) cur);
      }
      System.out.println();
   }
}

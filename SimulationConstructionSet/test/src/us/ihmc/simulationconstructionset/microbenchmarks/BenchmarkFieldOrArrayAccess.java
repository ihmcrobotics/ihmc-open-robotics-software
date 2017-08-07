package us.ihmc.simulationconstructionset.microbenchmarks;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import us.ihmc.commons.Conversions;

public class BenchmarkFieldOrArrayAccess
{

   private static final int ITERATIONS = 1000000;
   private static final int TESTS = 100;
   private static final boolean USE_COMMON_STORAGE = true;

   public static void main(String[] args)
   {
      new BenchmarkFieldOrArrayAccess();
   }

   private final long[] randomVariables = new long[ITERATIONS];
   private final Access[] testLongAccess = new Access[ITERATIONS];
   
   private final int[] accessArray = new int[ITERATIONS];

   public BenchmarkFieldOrArrayAccess()
   {
      Class<?>[] classes = { ArrayAccess.class, PrimitiveAccess.class };

      
      ArrayList<Integer> access = new ArrayList<Integer>();
      for(int i = 0; i < ITERATIONS; i++)
      {
         access.add(i);
      }
      Collections.shuffle(access);
      for(int i = 0; i < ITERATIONS; i++)
      {
         accessArray[i] = access.get(i);
      }
      
      String res = new String();
      for(Class<?> clazz : classes)
      {
         long writeTime = 0;
         long readTime = 0;
         for (int i = 0; i < TESTS; i++)
         {
            generateData(clazz);
            
            long start = System.nanoTime();
            testWriteAccess();
            if(i >= TESTS/2)
            {
               writeTime += System.nanoTime() - start;
            }
//            System.out.println("Writing " + ITERATIONS + " of " + clazz.getSimpleName() + " costs " + TimeTools.nanoSecondstoSeconds(System.nanoTime() - start) + "s");
            
            start = System.nanoTime();
            long testValue = testReadAccess();
//            System.out.println("Reading " + ITERATIONS + " of " + clazz.getSimpleName() + " costs " + TimeTools.nanoSecondstoSeconds(System.nanoTime() - start) + "s");
            if(i >= TESTS/2)
            {
               readTime += System.nanoTime() - start;
            }
            System.out.println("Test " + i + " returned " + testValue);
         }
         
         res += "Writing " + ITERATIONS + " of " + clazz.getSimpleName() + " averaged " + Conversions.nanosecondsToSeconds(writeTime/(TESTS - TESTS/2)) + "s" + System.lineSeparator();
         res += "Reading " + ITERATIONS + " of " + clazz.getSimpleName() + " averaged " + Conversions.nanosecondsToSeconds(readTime/(TESTS - TESTS/2)) + "s" + System.lineSeparator();
         
      }
      
      System.out.println(res);
   }

   private void generateData(Class<?> clazz)
   {
      Random random = new Random();

      for (int i = 0; i < randomVariables.length; i++)
      {
         randomVariables[i] = random.nextLong();
         try
         {
            testLongAccess[i] = (Access) clazz.newInstance();
         }
         catch (InstantiationException e)
         {
            throw new RuntimeException(e);
         }
         catch (IllegalAccessException e)
         {
            throw new RuntimeException(e);
         }
         random = new Random(); // Create trash between variables
         
      }
      if(USE_COMMON_STORAGE)
      {
         
         if(clazz == ArrayAccess.class)
         {
            long[] backingArray = new long[randomVariables.length];
            for (int i = 0; i < randomVariables.length; i++)
            {
               ((ArrayAccess)testLongAccess[i]).setBackingArray(backingArray, i);
            }
         }
         
      }
   }

   public void testWriteAccess()
   {
      for (int i = 0; i < randomVariables.length; i++)
      {
         testLongAccess[accessArray[i]].setValue(randomVariables[i]);
      }

   }

   public long testReadAccess()
   {

      long xor = 0;

      for (int i = 0; i < randomVariables.length; i++)
      {
         xor ^= testLongAccess[accessArray[i]].getValue();
      }

      return xor;
   }

   public interface Access
   {

      public abstract void setValue(long value);

      public abstract long getValue();

   }

   public static class PrimitiveAccess implements Access
   {
      private long value;

      @Override
      public void setValue(long value)
      {
         this.value = value;
      }

      @Override
      public long getValue()
      {
         return value;
      }
   }

   public static class ArrayAccess implements Access
   {
      private int index = 0;
      private long[] value = new long[1];

      @Override
      public void setValue(long value)
      {
         this.value[index] = value;
      }

      @Override
      public long getValue()
      {
         return value[index];
      }
      
      public void setBackingArray(long[] backingArray, int index)
      {
         this.value = backingArray;
         this.index = index;
      }
   }
}

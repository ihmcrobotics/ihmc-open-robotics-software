package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.Random;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TestLongBitsVsNativeDoubleSpeed
{
   public static void main(String[] args)
   {
      int numberOfVariables = 1000000;
      
      Random random = new Random();
      ArrayList<YoDouble> variables = new ArrayList<YoDouble>(numberOfVariables);
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      
      for(int i = 0; i < numberOfVariables; i++)
      {
         YoDouble v = new YoDouble("test_" + i, registry);
         v.set(random.nextDouble());
         variables.add(v);
      }
      
      
      ByteBuffer buffer = ByteBuffer.allocate(numberOfVariables * 8);
      DoubleBuffer doubleBuffer = buffer.asDoubleBuffer();
      LongBuffer longBuffer = buffer.asLongBuffer();
      
      for(int i = 0; i < 100; i++)
      {
         testDoubleBuffer(numberOfVariables, variables, doubleBuffer);
         testLongBuffer(numberOfVariables, variables, longBuffer);
         doubleBuffer.clear();
         longBuffer.clear();
      }
      
      
      long start = System.nanoTime();
      testDoubleBuffer(numberOfVariables, variables, doubleBuffer);
      double end = ((double) (System.nanoTime() - start))/1e6;
      System.out.println("Double buffer took " + end + " ms");
      
      
      start = System.nanoTime();
      testLongBuffer(numberOfVariables, variables, longBuffer);
      end = ((double) (System.nanoTime() - start))/1e6;
      System.out.println("Long buffer took " + end + " ms");
      
   }

   private static void testLongBuffer(int numberOfVariables, ArrayList<YoDouble> variables, LongBuffer longBuffer)
   {
      for(int i = 0; i < numberOfVariables; i++)
      {
         longBuffer.put(Double.doubleToLongBits(variables.get(i).getDoubleValue()));
      }
   }

   private static void testDoubleBuffer(int numberOfVariables, ArrayList<YoDouble> variables, DoubleBuffer doubleBuffer)
   {
      for(int i = 0; i < numberOfVariables; i++)
      {
         doubleBuffer.put(variables.get(i).getDoubleValue());
      }
   }
}

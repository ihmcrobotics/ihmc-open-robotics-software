package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.Random;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;



public class TestLongBitsVsNativeDoubleSpeed
{
   public static void main(String[] args)
   {
      int numberOfVariables = 1000000;
      
      Random random = new Random();
      ArrayList<DoubleYoVariable> variables = new ArrayList<DoubleYoVariable>(numberOfVariables);
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      
      for(int i = 0; i < numberOfVariables; i++)
      {
         DoubleYoVariable v = new DoubleYoVariable("test_" + i, registry);
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

   private static void testLongBuffer(int numberOfVariables, ArrayList<DoubleYoVariable> variables, LongBuffer longBuffer)
   {
      for(int i = 0; i < numberOfVariables; i++)
      {
         longBuffer.put(Double.doubleToLongBits(variables.get(i).getDoubleValue()));
      }
   }

   private static void testDoubleBuffer(int numberOfVariables, ArrayList<DoubleYoVariable> variables, DoubleBuffer doubleBuffer)
   {
      for(int i = 0; i < numberOfVariables; i++)
      {
         doubleBuffer.put(variables.get(i).getDoubleValue());
      }
   }
}

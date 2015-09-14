package us.ihmc.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotSame;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.Reader;
import java.io.StringReader;
import java.io.StringWriter;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ArrayToolsTest
{
   private static final double EPSILON = 1e-10;


	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseDoubleArrayFromMATLABString()
   {
      try
      {
         double[] expectedArray = generateDoubleArray();
         StringWriter stringWriter = new StringWriter();
         PrintWriter printWriter = new PrintWriter(stringWriter);
         printWriter.print(Arrays.toString(expectedArray));

         String matlabString = stringWriter.toString();
         double[] parsedArray = ArrayTools.parseDoubleArrayFromMATLAB(matlabString);

         for (int i = 0; i < expectedArray.length; i++)
            assertEquals(expectedArray[i], parsedArray[i], 0);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseDoubleArrayFromMATLABBufferedReader()
   {
      try
      {
         double[] expectedArray = generateDoubleArray();
         StringWriter stringWriter = new StringWriter();
         PrintWriter printWriter = new PrintWriter(stringWriter);
         printWriter.print(Arrays.toString(expectedArray));

         String matlabString = stringWriter.toString();
         BufferedReader reader = new BufferedReader(new StringReader(matlabString));
         double[] parsedArray = ArrayTools.parseDoubleArrayFromMATLAB(reader);

         for (int i = 0; i < expectedArray.length; i++)
            assertEquals(expectedArray[i], parsedArray[i], 0);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseDoubleArrayFromDataInputStream()
   {
      try
      {
         double[] expectedArray = generateDoubleArray();
         ByteArrayOutputStream byteOutStream = new ByteArrayOutputStream();
         DataOutputStream dataOutputStream = new DataOutputStream(byteOutStream);
         dataOutputStream.writeInt(expectedArray.length);
         for(double expected : expectedArray)
            dataOutputStream.writeDouble(expected);
         
         
         DataInputStream dataInputStream = new DataInputStream(new ByteArrayInputStream(byteOutStream.toByteArray()));
         double[] parsedArray = ArrayTools.parseDoubleArray(dataInputStream);

         for (int i = 0; i < expectedArray.length; i++)
            assertEquals(expectedArray[i], parsedArray[i], 0);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseIntegerArrayFromString()
   {
      try
      {
         int[] expectedArray = generateIntegerArray();
         StringWriter stringWriter = new StringWriter();

         String sourceString = Arrays.toString(expectedArray);
         int[] parsedArray = ArrayTools.parseIntegerArray(sourceString);

         for (int i = 0; i < expectedArray.length; i++)
            assertEquals(expectedArray[i], parsedArray[i]);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseIntegerArrayFromBufferedReader()
   {
      try
      {
         int[] expectedArray = generateIntegerArray();

         BufferedReader reader = new BufferedReader(new StringReader(Arrays.toString(expectedArray)));
         int[] parsedArray = ArrayTools.parseIntegerArray(reader);

         for (int i = 0; i < expectedArray.length; i++)
            assertEquals(expectedArray[i], parsedArray[i]);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseIntegerArrayFromDataInputStream()
   {
      try
      {
         int[] expectedArray = generateIntegerArray();
         ByteArrayOutputStream byteOutStream = new ByteArrayOutputStream();
         DataOutputStream dataOutputStream = new DataOutputStream(byteOutStream);
         dataOutputStream.writeInt(expectedArray.length);
         for(int expected : expectedArray)
            dataOutputStream.writeInt(expected);
         dataOutputStream.flush();

         DataInputStream dataInputStream = new DataInputStream(new ByteArrayInputStream(byteOutStream.toByteArray()));
         int[] parsedArray = ArrayTools.parseIntArray(dataInputStream);

         for (int i = 0; i < expectedArray.length; i++)
            assertEquals(expectedArray[i], parsedArray[i]);
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }


	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDeltaEquals()
   {
      double[] array1 = generateDoubleArray();
      double[] array2 = generateDoubleArray(array1.length);
      
      Double[] arrayOfDifferences = new Double[array1.length];
      
      for(int i = 0; i < array1.length; i++)
         arrayOfDifferences[i] = Math.abs(array1[i] - array2[i]);
      
      double largestDifference = (Double) (Collections.max(Arrays.asList(arrayOfDifferences)));
      System.out.println(largestDifference);
      assertTrue(ArrayTools.deltaEquals(array1, array2, largestDifference + 1));
      assertFalse(ArrayTools.deltaEquals(array1, array2, largestDifference - 1));
   }
	
	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	public void testDeltaEqualsWithNull()
	{
	   double[] array1 = null;
	   double[] array2 = null;
	   double[] array3 = generateDoubleArray();
	   double dummyDelta = 10e-2; // dummy value
	   
	   assertFalse(ArrayTools.deltaEquals(array1, array2, dummyDelta)); // 2 null arrays should return false
	   assertFalse(ArrayTools.deltaEquals(array1, array3, dummyDelta));
	}

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConcatentateArrays()
   {
      int x = 0;
      Object[][] arrayOfArraysToConcatenate = new Object[2][2];
      for (int i = 0; i < 2; i++)
      {
         for (int j = 0; j < 2; j++)
         {
            arrayOfArraysToConcatenate[i][j] = x;
            x++;
         }
      }

      Object[] expectedReturn = { 0, 1, 2, 3 };
      Object[] actualReturn = ArrayTools.concatentateArrays(arrayOfArraysToConcatenate);
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2])
            && (actualReturn[3] == expectedReturn[3]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConcatentateArrays1()
   {
      double x = 0.5;
      double[][] arrayOfArraysToConcatenate = new double[2][2];
      for (int i = 0; i < 2; i++)
      {
         for (int j = 0; j < 2; j++)
         {
            arrayOfArraysToConcatenate[i][j] = x;
            x++;
         }
      }

      double[] expectedReturn = { 0.5, 1.5, 2.5, 3.5 };
      double[] actualReturn = ArrayTools.concatentateArrays(arrayOfArraysToConcatenate);
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2])
            && (actualReturn[3] == expectedReturn[3]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConcatentateArrays2()
   {
      int x = 0;
      int[][] arrayOfArraysToConcatenate = new int[2][2];
      for (int i = 0; i < 2; i++)
      {
         for (int j = 0; j < 2; j++)
         {
            arrayOfArraysToConcatenate[i][j] = x;
            x++;
         }
      }

      int[] expectedReturn = { 0, 1, 2, 3 };
      int[] actualReturn = ArrayTools.concatentateArrays(arrayOfArraysToConcatenate);
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2])
            && (actualReturn[3] == expectedReturn[3]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyArray()
   {
      double[] array = { 0.2, -1.6 };
      double[] expectedReturn = { 0.2, -1.6 };
      double[] actualReturn = Arrays.copyOf(array, array.length);

      /** @todo fill in the test code */
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyArray1()
   {
      float[] array = { 10, -20, 30 };
      float[] expectedReturn = { 10, -20, 30 };
      float[] actualReturn = Arrays.copyOf(array, array.length);

      /** @todo fill in the test code */
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyArray2()
   {
      int[] array = { 1, -45, 5 };
      int[] expectedReturn = { 1, -45, 5 };
      int[] actualReturn = Arrays.copyOf(array, array.length);

      /** @todo fill in the test code */
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyArray3()
   {
      long[] array = { 18, -20, 7 };
      long[] expectedReturn = { 18, -20, 7 };
      long[] actualReturn = Arrays.copyOf(array, array.length);

      /** @todo fill in the test code */
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyGenericArray()
   {
      Apple apple = new Apple();
      Pear pear = new Pear();
      Orange orange = new Orange();
      Fruit[] fruits = {apple, pear, orange};
      Fruit[] copiedFruits = Arrays.copyOf(fruits, fruits.length);
      
      assertNotSame(fruits, copiedFruits);
      assertEquals(fruits.length, copiedFruits.length);
      for(int i = 0; i < fruits.length; i++)
      {
         assertSame(fruits[i], copiedFruits[i]);         
      }
      Orange orange2 = new Orange();
      fruits[1] = orange2;
      assertSame(copiedFruits[1], pear);
   }
   
   private class Fruit
   {}
   private class Apple extends Fruit
   {}
   private class Pear extends Fruit
   {}
   private class Orange extends Fruit
   {}

	@DeployableTestMethod(estimatedDuration = 1.2)
	@Test(timeout = 30000)
   public void testParseDoubleArrayFromString() throws IOException
   {
      Random random = new Random();
      int arraySize = random.nextInt(100) + 1;

      NumberFormat formatter = new DecimalFormat("0.000");

      for (int j = 0; j < 10000; j++)
      {
         double[] originalArrayOfRandomNumbers = new double[arraySize];

         StringBuilder stringBuilder = generateParseableArrayString(random, arraySize, formatter, originalArrayOfRandomNumbers);

         double[] parsedArray = null;

         parsedArray = ArrayTools.parseDoubleArray(stringBuilder.toString());

         for (int i = 0; i < arraySize; i++)
         {
            assertEquals(originalArrayOfRandomNumbers[i], parsedArray[i], 0.0);
         }
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseDoubleArrayFromBufferedReader()
   {
      Random random = new Random();
      int arraySize = random.nextInt(100) + 1;

      NumberFormat formatter = new DecimalFormat("0.000");
      //      formatter.setRoundingMode(RoundingMode.UNNECESSARY);

      for (int j = 0; j < 100; j++)
      {
         double[] originalArrayOfRandomNumbers = new double[arraySize];

         StringBuilder stringBuilder = generateParseableArrayString(random, arraySize, formatter, originalArrayOfRandomNumbers);

         double[] parsedArray = null;

         BufferedReader mockBufferedReader = new BufferedReader(new StringReader(stringBuilder.toString()));

         try
         {
            parsedArray = ArrayTools.parseDoubleArray(mockBufferedReader);
         }
         catch (IOException e)
         {
            e.printStackTrace();
            fail();
         }

         for (int i = 0; i < arraySize; i++)
         {
            assertEquals(originalArrayOfRandomNumbers[i], parsedArray[i], 0);
         }
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000 , expected = IOException.class)
   public void testParseDoubleArrayFromBufferedReaderWithIOException() throws IOException
   {
      BufferedReader mockBufferedReader = new BufferedReader(new Reader()
      {
         public int read(char[] cbuf, int off, int len) throws IOException
         {
            throw new IOException();
         }

         public void close() throws IOException
         {
            return;
         }
      });

      ArrayTools.parseDoubleArray(mockBufferedReader);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetArrayFromArrayList()
   {
      int numberOfTests = 5;
      for(int i = 0; i<numberOfTests; i++)
      {
         ArrayList<Double> arrayList = generateDoubleArrayList();
         
         @SuppressWarnings("unchecked")
         ArrayList<Double> arrayListCopy = (ArrayList<Double>) arrayList.clone();
         
         double[] array = ArrayTools.getArrayFromArrayList(arrayList);
         
         assertEquals(arrayList.size(), array.length);
         
         for(int j = 0; j<arrayList.size(); j++)
         {
            assertEquals(arrayList.get(j), array[j], 1e-8);
            assertEquals(arrayList.get(j), arrayListCopy.get(j), 1e-8);
         }
         
         
      }
      
      ArrayList<Double> arrayList = new ArrayList<Double>();
      double[] array = ArrayTools.getArrayFromArrayList(arrayList);
      
      assertEquals(arrayList.size(), array.length);
      
      
      arrayList = null;
      array = ArrayTools.getArrayFromArrayList(arrayList);
      
      assertNull(array);
   }

   private StringBuilder generateParseableArrayString(Random random, int arraySize, NumberFormat formatter, double[] originalArrayOfRandomNumbers)
   {
      for (int i = 0; i < arraySize; i++)
      {
         double nextNumber = Double.parseDouble(formatter.format(random.nextGaussian()));
         originalArrayOfRandomNumbers[i] = nextNumber;
      }

      StringBuilder stringBuilder = new StringBuilder();

      for (int i = 0; i < arraySize; i++)
      {
         stringBuilder.append(originalArrayOfRandomNumbers[i]);
         if (i != (arraySize - 1))
            stringBuilder.append(",");
      }
      return stringBuilder;
   }

   private String newLine()
   {
      return System.getProperty("line.separator");
   }

   private double[] generateDoubleArray()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] array = new double[arraySize];
      NumberFormat formatter = new DecimalFormat("0.000");

      for (int i = 0; i < arraySize; i++)
         array[i] = Double.parseDouble(formatter.format(random.nextGaussian()));

      return array;
   }
   
   private ArrayList<Double> generateDoubleArrayList()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      ArrayList<Double> array = new ArrayList<Double>();
      
      for (int i = 0; i < arraySize; i++)
         array.add((random.nextGaussian()));

      return array;
   }
   
   private double[] generateDoubleArray(int size)
   {
      Random random = new Random();
      int arraySize = size;
      double[] array = new double[arraySize];
      NumberFormat formatter = new DecimalFormat("0.000");

      for (int i = 0; i < arraySize; i++)
         array[i] = Double.parseDouble(formatter.format(random.nextGaussian()));

      return array;
   }
   
   private int[] generateIntegerArray()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      int[] array = new int[arraySize];

      for (int i = 0; i < arraySize; i++)
         array[i] = random.nextInt();

      return array;
   }
	
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetRearrangedArrayList()
   {
      ArrayList<Integer> arrayList = new ArrayList<Integer>();
      
      arrayList.add(1);
      arrayList.add(2);
      arrayList.add(3);
      arrayList.add(4);
      
      ArrayList<Integer> rearrangedList = ArrayTools.getRearrangedArrayListCopy(arrayList, 1);
      
      ArrayList<Integer> expectedList = new ArrayList<>();
      expectedList.add(2);
      expectedList.add(3);
      expectedList.add(4);
      expectedList.add(1);
      
      assertEquals(expectedList, rearrangedList);
      
      assertEquals(arrayList, ArrayTools.getRearrangedArrayListCopy(arrayList, 0));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetArrayListFromArray()
   {
      Random random = new Random(1561361L);
      int length = random.nextInt(1000);
      double amplitude = 100.0 * random.nextDouble();
      
      
      double[] array = new double[length];
      for(int i = 0; i < length; i++)
      {
         array[i] = -0.5 * amplitude + amplitude * random.nextDouble();
      }
      ArrayList<Double> arrayList = ArrayTools.getArrayListFromArray(array);
      
      for (int i = 0; i < array.length; i++)
         assertEquals(array[i], arrayList.get(i), EPSILON);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetMaximumAbsoluteChangeBetweenTicks()
   {
      double[] array = new double[]{1.7};
      double maxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(array);
      assertEquals(0.0, maxChange, 1e-7);
    
      array = new double[]{1.0, 2.0, 3.0};
      maxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(array);
      assertEquals(1.0, maxChange, 1e-7);
      
      array = new double[]{3.0, 2.0, 1.0};
      maxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(array);
      assertEquals(1.0, maxChange, 1e-7);
      
      array = new double[]{1.0, 1.01, 1.03, 1.04};
      maxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(array);
      assertEquals(0.02, maxChange, 1e-7);
      
      array = new double[]{1.0, 1.01, 1.02, 1.0};
      maxChange = ArrayTools.getMaximumAbsoluteChangeBetweenTicks(array);
      assertEquals(0.02, maxChange, 1e-7);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsContinuous()
   {
      double[] array = new double[]{1.7};
      assertTrue(ArrayTools.isContinuous(array, 1e-7));
      
      array = new double[]{1.0, 2.0, 3.0};
      assertTrue(ArrayTools.isContinuous(array, 1.0 + 1e-7));
      assertFalse(ArrayTools.isContinuous(array, 1.0 - 1e-7));
      
      array = new double[]{3.0, 2.0, 1.0};
      assertTrue(ArrayTools.isContinuous(array, 1.0 + 1e-7));
      assertFalse(ArrayTools.isContinuous(array, 1.0 - 1e-7));

      
      array = new double[]{1.0, 2.0, 3.0, 2.0, 1.0};
      assertTrue(ArrayTools.isContinuous(array, 1.0 + 1e-7));
      assertFalse(ArrayTools.isContinuous(array, 1.0 - 1e-7));
      
      array = new double[]{1.0, 1.01, 1.03, 1.04};
      assertTrue(ArrayTools.isContinuous(array, 0.02 + 1e-7));
      assertFalse(ArrayTools.isContinuous(array, 0.02 - 1e-7));
   }
}

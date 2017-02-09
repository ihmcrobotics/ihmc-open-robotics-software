package us.ihmc.tools;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Reader;
import java.io.StringReader;
import java.io.StringWriter;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ArrayToolsTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseIntegerArrayFromString()
   {
      try
      {
         int[] expectedArray = generateIntegerArray();
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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


	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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
	
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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




	@ContinuousIntegrationTest(estimatedDuration = 1.4)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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
	
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
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

	
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

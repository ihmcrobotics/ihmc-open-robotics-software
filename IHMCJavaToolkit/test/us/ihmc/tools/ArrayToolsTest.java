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

import us.ihmc.tools.ArrayTools;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class ArrayToolsTest
{
   private static final double EPSILON = 1e-10;

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfLongs()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      long[] originalArrayOfLongs = new long[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
      {
         originalArrayOfLongs[i] = random.nextLong();
         stringBuilder.append(originalArrayOfLongs[i]);
         stringBuilder.append(", ");
      }

      stringBuilder.append("}");

      StringWriter writer = new StringWriter();
      ArrayTools.printArrayOfLongs(originalArrayOfLongs, writer);
      assertEquals(stringBuilder.toString(), writer.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayNumbersAsStrings()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      String[] arrayElements = new String[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
         arrayElements[i] = formatter.format(random.nextGaussian());

      for (String s : arrayElements)
         stringBuilder.append(s + ", ");

      stringBuilder.append("}\n");

      StringWriter writer = new StringWriter();

      ArrayTools.printArray(arrayElements, writer);

      assertEquals(stringBuilder.toString(), writer.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfInts()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      int[] originalArrayOfLongs = new int[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
      {
         originalArrayOfLongs[i] = random.nextInt();
         stringBuilder.append(originalArrayOfLongs[i]);
         stringBuilder.append(", ");
      }

      stringBuilder.append("}\n");

      StringWriter writer = new StringWriter();
      ArrayTools.printArray(originalArrayOfLongs, writer);
      assertEquals(stringBuilder.toString(), writer.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfDoubles()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));

      for (double num : arrayElements)
         stringBuilder.append(num + ", ");

      stringBuilder.append("}\n");

      StringWriter writer = new StringWriter();

      ArrayTools.printArray(arrayElements, writer);

      assertEquals(stringBuilder.toString(), writer.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayFromStringsToPrintWriter()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      String[] arrayElements = new String[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
         arrayElements[i] = formatter.format(random.nextGaussian());

      for (String s : arrayElements)
         stringBuilder.append(s + ", ");

      stringBuilder.append("}" + newLine());

      StringWriter stringWriter = new StringWriter();
      PrintWriter printWriter = new PrintWriter(stringWriter);

      ArrayTools.printArray(arrayElements, printWriter);

      printWriter.flush();

      assertEquals(stringBuilder.toString(), stringWriter.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayFromStringsToPrintStream()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      String[] arrayElements = new String[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
         arrayElements[i] = formatter.format(random.nextGaussian());

      for (String s : arrayElements)
         stringBuilder.append(s + ", ");

      stringBuilder.append("}" + newLine());

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      ArrayTools.printArray(arrayElements, printStream);

      printStream.flush();

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayToStandardOutFromArrayList()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      ArrayList<String> arrayElements = new ArrayList<String>(arraySize);
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      String separator = (arraySize * 5) < 100 ? ", " : "\n";

      stringBuilder.append("[");

      for (int i = 0; i < arraySize; i++)
         arrayElements.add(i, formatter.format(Math.abs(random.nextGaussian())));

      for (String s : arrayElements)
         stringBuilder.append(s + separator);

      stringBuilder.append("]" + newLine());

      PrintStream stdout = System.out;

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      System.setOut(printStream);
      ArrayTools.printArray(arrayElements, null);
      printStream.flush();

      System.setOut(stdout);

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfDoublesForMATLABWithPrintWriter()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("[");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("]");

      StringWriter stringWriter = new StringWriter();
      PrintWriter printWriter = new PrintWriter(stringWriter);

      ArrayTools.printArrayForMATLAB(arrayElements, printWriter);

      printWriter.flush();

      assertEquals(stringBuilder.toString(), stringWriter.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfDoublesFOrMATLABWithPrintStream()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("[");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("]");

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      ArrayTools.printArrayForMATLAB(arrayElements, printStream);

      printStream.flush();

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintNamedArrayForMATLABWithPrintWriter()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      String arrayName = "testArray";

      stringBuilder.append(arrayName + " = [");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("]" + newLine());

      StringWriter stringWriter = new StringWriter();
      PrintWriter printWriter = new PrintWriter(stringWriter);

      ArrayTools.printArrayForMATLAB(arrayName, arrayElements, printWriter);

      printWriter.flush();

      assertEquals(stringBuilder.toString(), stringWriter.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintNamedArrayForMATLABWithPrintStream()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      String arrayName = "testArray";

      stringBuilder.append(arrayName + " = [");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("]" + newLine());

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      ArrayTools.printArrayForMATLAB(arrayName, arrayElements, printStream);

      printStream.flush();

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfDoublesToDataOutputStream()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = random.nextGaussian();

      }

      ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
      DataOutputStream dataOutputStream = new DataOutputStream(byteStream);

      try
      {
         ArrayTools.printArray(arrayElements, dataOutputStream);
         dataOutputStream.flush();
         ByteBuffer buffer = ByteBuffer.wrap(byteStream.toByteArray());

         assertEquals(arrayElements.length, buffer.getInt());

         int i = 0;
         while (buffer.hasRemaining())
            assertEquals(arrayElements[i++], buffer.getDouble(), 0);

         dataOutputStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfDoublesWithPrintWriter()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("}" + newLine());

      StringWriter stringWriter = new StringWriter();
      PrintWriter printWriter = new PrintWriter(stringWriter);

      ArrayTools.printArray(arrayElements, printWriter);

      printWriter.flush();

      assertEquals(stringBuilder.toString(), stringWriter.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testArrayToString()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      assertEquals(stringBuilder.toString(), ArrayTools.arrayToString(arrayElements));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintDoubleArrayWithPrintStream()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      double[] arrayElements = new double[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = Double.parseDouble(formatter.format(random.nextGaussian()));
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("}" + newLine());

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      ArrayTools.printArray(arrayElements, printStream);

      printStream.flush();

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testPrintTwoDimensionalArrayOfDoublesWithPrintStream()
   {
      Random random = new Random();
      int arraySizeRows = random.nextInt(500) + 1;
      int arraySizeCols = random.nextInt(500) + 1;
      double[][] arrayElements = new double[arraySizeRows][arraySizeCols];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      stringBuilder.append("{" + newLine());

      for (int i = 0; i < arraySizeRows; i++)
      {
         stringBuilder.append("{");
         for (int j = 0; j < arraySizeCols; j++)
         {
            arrayElements[i][j] = Double.parseDouble(formatter.format(random.nextGaussian()));
            stringBuilder.append(arrayElements[i][j]);
            if (j < arraySizeCols - 1)
               stringBuilder.append(", ");
         }
         stringBuilder.append("}" + newLine());
      }

      stringBuilder.append("}" + newLine());

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      ArrayTools.printArray(arrayElements, printStream);

      printStream.flush();

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfIntsToPrintWriter()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      int[] arrayElements = new int[arraySize];
      StringBuilder stringBuilder = new StringBuilder();

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = random.nextInt();
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("}" + newLine());

      StringWriter stringWriter = new StringWriter();
      PrintWriter printWriter = new PrintWriter(stringWriter);

      ArrayTools.printArray(arrayElements, printWriter);

      printWriter.flush();

      assertEquals(stringBuilder.toString(), stringWriter.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfIntsToPrintStream()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      int[] arrayElements = new int[arraySize];
      StringBuilder stringBuilder = new StringBuilder();

      stringBuilder.append("{");

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = random.nextInt();
         stringBuilder.append(arrayElements[i]);
         if (i < arraySize - 1)
            stringBuilder.append(", ");
      }

      stringBuilder.append("}" + newLine());

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      ArrayTools.printArray(arrayElements, printStream);

      printStream.flush();

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintArrayOfIntsToDataOutputStream()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      int[] arrayElements = new int[arraySize];

      for (int i = 0; i < arraySize; i++)
      {
         arrayElements[i] = random.nextInt();

      }

      ByteArrayOutputStream byteStream = new ByteArrayOutputStream();
      DataOutputStream dataOutputStream = new DataOutputStream(byteStream);

      try
      {
         ArrayTools.printArray(arrayElements, dataOutputStream);
         dataOutputStream.flush();
         ByteBuffer buffer = ByteBuffer.wrap(byteStream.toByteArray());

         assertEquals(arrayElements.length, buffer.getInt());

         int i = 0;
         while (buffer.hasRemaining())
            assertEquals(arrayElements[i++], buffer.getInt(), 0);

         dataOutputStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
         fail();
      }
   }

	@DeployableTestMethod(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testPrintTwoDimensionalArrayOfIntsToPrintStream()
   {
      Random random = new Random();
      int arraySizeRows = random.nextInt(500) + 1;
      int arraySizeCols = random.nextInt(500) + 1;
      int[][] arrayElements = new int[arraySizeRows][arraySizeCols];
      StringBuilder stringBuilder = new StringBuilder();

      stringBuilder.append("{" + newLine());

      for (int i = 0; i < arraySizeRows; i++)
      {
         stringBuilder.append("{");
         for (int j = 0; j < arraySizeCols; j++)
         {
            arrayElements[i][j] = random.nextInt();
            stringBuilder.append(arrayElements[i][j]);
            if (j < arraySizeCols - 1)
               stringBuilder.append(", ");
         }
         stringBuilder.append("}" + newLine());
      }

      stringBuilder.append("}" + newLine());

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      ArrayTools.printIntArray(arrayElements, printStream);

      printStream.flush();

      assertEquals(stringBuilder.toString(), outputStream.toString());
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testParseDoubleArrayFromMATLABString()
   {
      try
      {
         double[] expectedArray = generateDoubleArray();
         StringWriter stringWriter = new StringWriter();
         PrintWriter printWriter = new PrintWriter(stringWriter);
         ArrayTools.printArrayForMATLAB(expectedArray, printWriter);

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
         ArrayTools.printArrayForMATLAB(expectedArray, printWriter);

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
         ArrayTools.printArray(expectedArray, dataOutputStream);

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
         PrintWriter printWriter = new PrintWriter(stringWriter);
         ArrayTools.printArray(expectedArray, printWriter);

         String sourceString = stringWriter.toString();
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
         StringWriter stringWriter = new StringWriter();
         PrintWriter printWriter = new PrintWriter(stringWriter);
         ArrayTools.printArray(expectedArray, printWriter);

         BufferedReader reader = new BufferedReader(new StringReader(stringWriter.toString()));
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
         ArrayTools.printArray(expectedArray, dataOutputStream);

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
   public void testGetReverseOrderedArrayCopy()
   {
      double[] array = generateDoubleArray();
      double[] reversedCopyOfArray = ArrayTools.getReserveredOrderedArrayCopy(array);
      
      for(int i = 0; i < array.length; i++)
         assertEquals(array[array.length - i - 1], reversedCopyOfArray[i], 0);
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrintGenericArray()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      String[] arrayElements = new String[arraySize];
      StringBuilder stringBuilder = new StringBuilder();
      NumberFormat formatter = new DecimalFormat("0.000");

      String separator = (arraySize * 5) < 100 ? ", " : "\n";

      stringBuilder.append("[");

      for (int i = 0; i < arraySize; i++)
         arrayElements[i] = formatter.format(Math.abs(random.nextGaussian()));

      for (String s : arrayElements)
         stringBuilder.append(s + separator);

      stringBuilder.append("]" + newLine());

      PrintStream stdout = System.out;

      ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
      PrintStream printStream = new PrintStream(outputStream);

      System.setOut(printStream);
      ArrayTools.printArray(arrayElements);
      printStream.flush();

      System.setOut(stdout);

      assertEquals(stringBuilder.toString(), outputStream.toString());
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
      double[] actualReturn = ArrayTools.copyArray(array);

      /** @todo fill in the test code */
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyArray1()
   {
      float[] array = { 10, -20, 30 };
      float[] expectedReturn = { 10, -20, 30 };
      float[] actualReturn = ArrayTools.copyArray(array);

      /** @todo fill in the test code */
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyArray2()
   {
      int[] array = { 1, -45, 5 };
      int[] expectedReturn = { 1, -45, 5 };
      int[] actualReturn = ArrayTools.copyArray(array);

      /** @todo fill in the test code */
      assertTrue("Test Failed", (actualReturn[0] == expectedReturn[0]) && (actualReturn[1] == expectedReturn[1]) && (actualReturn[2] == expectedReturn[2]));
   }

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCopyArray3()
   {
      long[] array = { 18, -20, 7 };
      long[] expectedReturn = { 18, -20, 7 };
      long[] actualReturn = ArrayTools.copyArray(array);

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
      Fruit[] copiedFruits = ArrayTools.copyArray(fruits);
      
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
   public void testGetReversedArrayList()
   {
      ArrayList<Integer> arrayList = new ArrayList<Integer>();
      ArrayList<Integer> arrayList2 = new ArrayList<Integer>();
      arrayList.add(1);
      arrayList.add(2);
      arrayList.add(3);

      arrayList2.add(3);
      arrayList2.add(2);
      arrayList2.add(1);

      ArrayList<Integer> expectedReturn = arrayList2;
      ArrayList<Integer> actualReturn = ArrayTools.getReversedArrayList(arrayList);
      assertEquals("return value", expectedReturn, actualReturn);

      /** @todo fill in the test code */
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
      int length = RandomTools.generateRandomInt(random, 0, 1000);
      double amplitude = RandomTools.generateRandomDouble(random, 0.0, 100.0);
      double[] array = RandomTools.generateRandomDoubleArray(random, length, amplitude);
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

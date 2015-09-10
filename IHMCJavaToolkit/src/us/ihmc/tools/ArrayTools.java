package us.ihmc.tools;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.StringTokenizer;

public class ArrayTools
{
   /**
    * Deprecated, use stringWriter.println(Arrays.toString(array))
    * 
    * Prints an Array of Longs
    * with StringWriter
    *
    * @param longs long[]
    * @param stringWriter StringWriter
    */
   @Deprecated
   public static void printArrayOfLongs(long[] longs, StringWriter stringWriter)
   {
      stringWriter.append("{");

      for (long aLong : longs)
      {
         stringWriter.append(aLong + ", ");
      }

      stringWriter.append("}");
   }

   /**
    * Deprecated, use stringWriter.println(Arrays.toString(array))
    * 
    * Prints an Array of strings
    * with StringWriter
    *
    * @param strings String[]
    * @param stringWriter StringWriter
    */
   @Deprecated
   public static void printArray(String[] strings, StringWriter stringWriter)
   {
      stringWriter.append("{");

      for (String string : strings)
      {
         stringWriter.append(string + ", ");
      }

      stringWriter.append("}\n");
   }

   /**
    * Deprecated, use stringWriter.println(Arrays.toString(array))
    * 
    * Prints an Array of integers
    * with StringWriter
    *
    * @param ints int[]
    * @param stringWriter StringWriter
    */
   @Deprecated
   public static void printArray(int[] ints, StringWriter stringWriter)
   {
      stringWriter.append("{");

      for (int i : ints)
      {
         stringWriter.append(i + ", ");
      }

      stringWriter.append("}\n");
   }

   /**
    * Deprecated, use stringWriter.println(Arrays.toString(array))
    * 
    * Prints an Array of Doubles
    * with StringWriter
    *
    * @param doubles double[]
    * @param stringWriter StringWriter
    */
   @Deprecated
   public static void printArray(double[] doubles, StringWriter stringWriter)
   {
      stringWriter.append("{");

      for (double d : doubles)
      {
         stringWriter.append(d + ", ");
      }

      stringWriter.append("}\n");
   }

   /**
    * Deprecated, use stringWriter.println(Arrays.toString(array))
    * 
    * Prints an Array of strings
    * with PrintWriter
    *
    * @param strings String[]
    * @param printWriter PrintWriter
    */
   @Deprecated
   public static void printArray(String[] strings, PrintWriter printWriter)
   {
      printWriter.print("{");

      for (String string : strings)
      {
         printWriter.print(string + ", ");
      }

      printWriter.println("}");
   }

   /**
    * Deprecated, use stringWriter.println(Arrays.deepToString(array))
    * 
    * Prints an Array of Stings
    * with PrintStream
    *
    * @param strings String[]
    * @param printStream PrintStream
    */
   @Deprecated
   public static void printArray(String[] strings, PrintStream printStream)
   {
      PrintWriter printWriter = new PrintWriter(printStream);
      printArray(strings, printWriter);
      printWriter.flush();
   }

   public static <T> void printArray(ArrayList<T> arrayList, PrintStream printStream)
   {
      StringBuilder builder = new StringBuilder();

      int maxCharsForOneLine = 100;
      int approximateNumberOfChars = arrayList.get(0).toString().length() * arrayList.size();
      String separator = (approximateNumberOfChars < maxCharsForOneLine) ? ", " : "\n";

      builder.append("[");

      for (T t : arrayList)
      {
         builder.append(t.toString() + separator);
      }

      builder.append("]");
      System.out.println(builder.toString());
   }

   /**
    * Prints an Array of doubles for MATLAB
    * with PrintWriter
    *
    * @param doubles double[]
    * @param printWriter PrintWriter
    */
   public static void printArrayForMATLAB(double[] doubles, PrintWriter printWriter)
   {
      printWriter.print("[");

      for (int i = 0; i < doubles.length; i++)
      {
         printWriter.print(doubles[i]);
         if (i < (doubles.length - 1))
            printWriter.print(", ");
      }

      printWriter.print("]");
   }

   /**
    * Prints an Array of doubles for MATLAB
    * with PrintStream
    *
    * @param doubles double[]
    * @param printStream PrintStream
    */
   public static void printArrayForMATLAB(double[] doubles, PrintStream printStream)
   {
      PrintWriter printWriter = new PrintWriter(printStream);
      printArrayForMATLAB(doubles, printWriter);
      printWriter.flush();
   }

   /**
    * Prints an Array of doubles for MATLAB
    * with PrintWriter
    *
    * @param arrayName String
    * @param doubles double[]
    * @param printWriter PrintWriter
    */
   public static void printArrayForMATLAB(String arrayName, double[] doubles, PrintWriter printWriter)
   {
      printWriter.print(arrayName + " = ");
      printArrayForMATLAB(doubles, printWriter);
      printWriter.println();
   }

   /**
    * Prints an Array for MATLAB
    * with PrintStream
    *
    * @param arrayName String
    * @param doubles double[]
    * @param printStream PrintStream
    */
   public static void printArrayForMATLAB(String arrayName, double[] doubles, PrintStream printStream)
   {
      PrintWriter printWriter = new PrintWriter(printStream);
      printArrayForMATLAB(arrayName, doubles, printWriter);
      printWriter.flush();
   }

   /**
    * Prints an Array of doubles
    * with DataOutputStream
    *
    * @param doubles double[]
    * @param dataOutputStream DataOutputStream
    * @throws IOException
    */
   public static void printArray(double[] doubles, DataOutputStream dataOutputStream) throws IOException
   {
      dataOutputStream.writeInt(doubles.length);

      for (int i = 0; i < doubles.length; i++)
      {
         dataOutputStream.writeDouble(doubles[i]);
      }
   }

   /**
    * Prints an Array of doubles
    * with PrintWriter
    *
    * @param doubles double[]
    * @param printWriter PrintWriter
    */
   public static void printArray(double[] doubles, PrintWriter printWriter)
   {
      printWriter.print("{");

      for (int i = 0; i < doubles.length; i++)
      {
         printWriter.print(doubles[i]);
         if (i < (doubles.length - 1))
            printWriter.print(", ");
      }

      printWriter.println("}");
   }

   public static String arrayToString(double[] doubles)
   {
      StringBuilder stringBuilder = new StringBuilder();

      for (int i = 0; i < doubles.length; i++)
      {
         stringBuilder.append(doubles[i]);
         if (i < (doubles.length - 1))
            stringBuilder.append(", ");
      }

      return stringBuilder.toString();
   }

   /**
    * Prints an Array of doubles
    * with PrintStream
    *
    * @param doubles double[]
    * @param printStream PrintStream
    */
   public static void printArray(double[] doubles, PrintStream printStream)
   {
      PrintWriter printWriter = new PrintWriter(printStream);
      printArray(doubles, printWriter);
      printWriter.flush();
   }

   /**
    *
    * @param doubles double[][]
    * @param printStream PrintStream
    */
   public static void printArray(double[][] doubles, PrintStream printStream)
   {
      printStream.println("{");

      for (int i = 0; i < doubles.length; i++)
      {
         printStream.print("{");

         for (int j = 0; j < doubles[i].length; j++)
         {
            printStream.print(doubles[i][j]);
            if (j < (doubles[i].length - 1))
               printStream.print(", ");
         }

         printStream.println("}");
      }

      printStream.println("}");
   }

   /**
    * Prints an Array of integers
    * with PrintWriter
    *
    * @param ints int[]
    * @param printWriter PrintWriter
    */
   public static void printArray(int[] ints, PrintWriter printWriter)
   {
      printWriter.print("{");

      for (int i = 0; i < ints.length; i++)
      {
         printWriter.print(ints[i]);
         if (i < (ints.length - 1))
            printWriter.print(", ");
      }

      printWriter.println("}");
   }

   /**
    * Prints an Array of intergers
    * with PrintStream
    *
    * @param ints int[]
    * @param printStream PrintStream
    */
   public static void printArray(int[] ints, PrintStream printStream)
   {
      PrintWriter printWriter = new PrintWriter(printStream);
      printArray(ints, printWriter);
      printWriter.flush();
   }

   /**
    * Prints an Array of integers
    * with DataOutputStream
    *
    * @param ints int[]
    * @param dataOutputStream DataOutputStream
    * @throws IOException
    */
   public static void printArray(int[] ints, DataOutputStream dataOutputStream) throws IOException
   {
      dataOutputStream.writeInt(ints.length);

      for (int i = 0; i < ints.length; i++)
      {
         dataOutputStream.writeInt(ints[i]);
      }
   }

   /**
    *
    * @param ints int[][]
    * @param printStream PrintStream
    */
   public static void printIntArray(int[][] ints, PrintStream printStream)
   {
      printStream.println("{");

      for (int i = 0; i < ints.length; i++)
      {
         printStream.print("{");

         for (int j = 0; j < ints[i].length; j++)
         {
            printStream.print(ints[i][j]);
            if (j < (ints[i].length - 1))
               printStream.print(", ");
         }

         printStream.println("}");
      }

      printStream.println("}");
   }

   //
   // Reading in Arrays
   //

   /**
    *
    * @param stringSource String
    * @return double[]
    * @throws IOException
    */
   public static double[] parseDoubleArray(String stringSource) throws IOException
   {
      String line = stringSource;
      ArrayList<Double> retArray = new ArrayList<Double>();

      // System.err.println("PrintAndParseArrayTools::parseDoubleArray: line : "
      // + line);
      if (line == null)
      {
         return null;
      }

      StringTokenizer tokenizer = new StringTokenizer(line.replace("{", "").replace("}", "").replace(" ", "").replace(";", ""), ",");
      while (tokenizer.hasMoreElements())
      {
         retArray.add(new Double(Double.parseDouble((String) tokenizer.nextElement())));

         // retArray.add(new
         // Double(Double.parseDouble(tokenizer.nextToken())));
      }

      double[] ret = new double[retArray.size()];

      // System.err.println("PrintAndParseArrayTools::parseDoubleArray: retArray
      // : " + retArray);
      for (int i = 0; i < retArray.size(); i++)
      {
         ret[i] = retArray.get(i);
      }

      return ret;
   }

   /**
    *
    * @param bufferedReader BufferedReader
    * @return double[]
    * @throws IOException
    */
   public static double[] parseDoubleArray(BufferedReader bufferedReader) throws IOException
   {
      String line = bufferedReader.readLine();

      return parseDoubleArray(line);
   }

   /**
    *
    * @param stringSource String
    * @return double[]
    * @throws IOException
    */
   public static double[] parseDoubleArrayFromMATLAB(String stringSource) throws IOException
   {
      String line = stringSource;
      ArrayList<Double> retArray = new ArrayList<Double>();
      if (line == null)
      {
         return null;
      }

      StringTokenizer tokenizer = new StringTokenizer(line.replace("[", "").replace("]", "").replace(" ", ""), ",");
      while (tokenizer.hasMoreElements())
      {
         retArray.add(new Double(Double.parseDouble((String) tokenizer.nextElement())));

         // retArray.add(new
         // Double(Double.parseDouble(tokenizer.nextToken())));
      }

      double[] ret = new double[retArray.size()];

      // System.err.println("PrintAndParseArrayTools::parseDoubleArray: retArray
      // : " + retArray);
      for (int i = 0; i < retArray.size(); i++)
      {
         ret[i] = retArray.get(i);
      }

      return ret;
   }

   /**
    *
    * @param bufferedReader BufferedReader
    * @return double[]
    * @throws IOException
    */
   public static double[] parseDoubleArrayFromMATLAB(BufferedReader bufferedReader) throws IOException
   {
      String line = bufferedReader.readLine();

      return parseDoubleArrayFromMATLAB(line);
   }

   public static double[] parseDoubleArray(DataInputStream dataInputStream) throws IOException
   {
      int numElements = dataInputStream.readInt();
      double[] ret = new double[numElements];
      for (int i = 0; i < numElements; i++)
      {
         ret[i] = dataInputStream.readDouble();
      }

      return ret;
   }

   public static int[] parseIntegerArray(String stringSource) throws IOException
   {
      ArrayList<Integer> retArray = new ArrayList<Integer>();
      String line = stringSource;
      if (line == null)
      {
         return null;
      }

      StringTokenizer tokenizer = new StringTokenizer(line.replace("{", "").replace("}", "").replace(" ", "").replace(System.getProperty("line.separator"),
                                     "").replace("\n", ""), ",");
      while (tokenizer.hasMoreElements())
      {
         retArray.add(new Integer(Integer.parseInt((String) tokenizer.nextElement())));
      }

      int[] ret = new int[retArray.size()];
      for (int i = 0; i < retArray.size(); i++)
      {
         ret[i] = retArray.get(i);
      }

      return ret;
   }

   public static int[] parseIntegerArray(BufferedReader bufferedReader) throws IOException
   {
      String line = bufferedReader.readLine();

      return parseIntegerArray(line);
   }

   public static int[] parseIntArray(DataInputStream dataInputStream) throws IOException
   {
      int numElements = dataInputStream.readInt();
      int[] ret = new int[numElements];
      for (int i = 0; i < numElements; i++)
      {
         ret[i] = dataInputStream.readInt();
      }

      return ret;
   }

   /**
    * Use Arrays.copyOf
    * 
    * Copys an Array of Floats
    *
    * @param array float[]
    * @return float[]
    */
   @Deprecated
   public static float[] copyArray(float[] array)
   {
      float[] ret = new float[array.length];
      System.arraycopy(array, 0, ret, 0, array.length);

      return ret;
   }

   /**
    * Copys an Array of Doubles
    *
    * @param array double[]
    * @return double[]
    */
   public static double[] copyArray(double[] array)
   {
      double[] ret = new double[array.length];
      System.arraycopy(array, 0, ret, 0, array.length);

      return ret;
   }

   /**
    * Copys an Array of intgers
    *
    * @param array int[]
    * @return int[]
    */
   public static int[] copyArray(int[] array)
   {
      int[] ret = new int[array.length];
      System.arraycopy(array, 0, ret, 0, array.length);

      return ret;
   }

   public static <T> T[] copyArray(T[] array)
   {
      T[] ret = Arrays.copyOf(array, array.length);

      return ret;
   }


	@Deprecated
   public static double[] getReserveredOrderedArrayCopy(double[] arrayToReverseAndCopy)
   {
      int length = arrayToReverseAndCopy.length;
      double[] ret = new double[length];

      for (int i = 0; i < length; i++)
      {
         ret[i] = arrayToReverseAndCopy[length - i - 1];
      }

      return ret;
   }

   /**
    * Copys an Array of Longs
    *
    * @param array long[]
    * @return long[]
    */
   public static long[] copyArray(long[] array)
   {
      long[] ret = new long[array.length];
      System.arraycopy(array, 0, ret, 0, array.length);

      return ret;
   }

   /**
    * Use ArrayUtils.addAll
    *
    * @param arrayOfArraysToConcatenate int[][]
    * @return int[]
    */
   @Deprecated
   public static int[] concatentateArrays(int[][] arrayOfArraysToConcatenate)
   {
      int retSize = 0;
      for (int i = 0; i < arrayOfArraysToConcatenate.length; i++)
      {
         retSize += arrayOfArraysToConcatenate[i].length;
      }

      int[] ret = new int[retSize];
      int currentIndexIntoRet = 0;
      for (int i = 0; i < arrayOfArraysToConcatenate.length; i++)
      {
         for (int j = 0; j < arrayOfArraysToConcatenate[i].length; j++)
         {
            ret[currentIndexIntoRet++] = arrayOfArraysToConcatenate[i][j];
         }
      }

      return ret;
   }

   /**
    * Use ArrayUtils.addAll
    *
    * @param arrayOfArraysToConcatenate double[][]
    * @return double[]
    */
   @Deprecated
   public static double[] concatentateArrays(double[][] arrayOfArraysToConcatenate)
   {
      int retSize = 0;
      for (int i = 0; i < arrayOfArraysToConcatenate.length; i++)
      {
         retSize += arrayOfArraysToConcatenate[i].length;
      }

      double[] ret = new double[retSize];
      int currentIndexIntoRet = 0;
      for (int i = 0; i < arrayOfArraysToConcatenate.length; i++)
      {
         for (int j = 0; j < arrayOfArraysToConcatenate[i].length; j++)
         {
            ret[currentIndexIntoRet++] = arrayOfArraysToConcatenate[i][j];
         }
      }

      return ret;
   }

   /**
    * Use ArrayUtils.addAll
    *
    * @param arrayOfArraysToConcatenate Object[][]
    * @return Object[]
    */
   @Deprecated
   public static Object[] concatentateArrays(Object[][] arrayOfArraysToConcatenate)
   {
      int retSize = 0;
      for (int i = 0; i < arrayOfArraysToConcatenate.length; i++)
      {
         retSize += arrayOfArraysToConcatenate[i].length;
      }

      Object[] ret = new Object[retSize];
      int currentIndexIntoRet = 0;
      for (int i = 0; i < arrayOfArraysToConcatenate.length; i++)
      {
         for (int j = 0; j < arrayOfArraysToConcatenate[i].length; j++)
         {
            ret[currentIndexIntoRet++] = arrayOfArraysToConcatenate[i][j];
         }
      }

      return ret;
   }

   public static <T> void printArray(T[] array)
   {
      // FIXME: Needs to Handle Null Arrays and Test it.
      StringBuilder builder = new StringBuilder();

      int maxCharsForOneLine = 100;
      int approximateNumberOfChars = array[0].toString().length() * array.length;
      String separator = (approximateNumberOfChars < maxCharsForOneLine) ? ", " : "\n";

      builder.append("[");

      for (T t : array)
      {
         builder.append(t.toString() + separator);
      }

      builder.append("]");
      System.out.println(builder.toString());
   }

   /**
    * Use JUnitTools.assertArraysEquals
    */
   @Deprecated
   public static boolean deltaEquals(double[] a, double[] b, double delta)
   {
      if(a == null || b == null)
         return false;
      
      if (a.length != b.length)
         return false;

      for (int i = 0; i < a.length; i++)
      {
         if (Math.abs(a[i] - b[i]) > delta)
            return false;
      }

      return true;
   }
   
   /**
    * Copies @param list, splits the list in half at @param newStartIndex, and then pastes the top half under the top half
    * 
    * @param list
    * @param newStartIndex
    * @return
    */
   public static <T> ArrayList<T> getRearrangedArrayListCopy(ArrayList<T> list, int newStartIndex)
   {
      ArrayList<T> ret = new ArrayList<T>();

      int listLength = list.size();
      
      for (int i = newStartIndex; i < listLength; i++)
         ret.add(list.get(i));

      for (int i = 0; i < newStartIndex; i++)
         ret.add(list.get(i));

      return ret;
   }

   /**
    * Returns the Array in Reverse Order
    *
    * @param arrayList ArrayList
    * @return ArrayList
    */
   public static <T> ArrayList<T> getReversedArrayList(ArrayList<T> arrayList)
   {
      ArrayList<T> ret = new ArrayList<T>();
      for (int i = arrayList.size() - 1; i >= 0; i--)
      {
         ret.add(arrayList.get(i));
      }

      return ret;
   }

   public static ArrayList<Double> getArrayListFromArray(double[] array)
   {
      ArrayList<Double> ret = new ArrayList<Double>();

      for (int i = 0; i < array.length; i++)
      {
         ret.add(array[i]);
      }

      return ret;
   }

   public static double[] getArrayFromArrayList(List<Double> arrayList)
   {
      // TODO: rename this method getArrayFromList
      if (arrayList == null)
         return null;

      double[] ret = new double[arrayList.size()];

      for (int i = 0; i < arrayList.size(); i++)
      {
         ret[i] = arrayList.get(i);
      }

      return ret;
   }
   
   public static double getMaximumAbsoluteChangeBetweenTicks(double[] values)
   {
      int length = values.length;
      double maxChange = 0.0;
      
      double previousValue = values[0];
      for (int i=1; i<length; i++)
      {
         double nextValue = values[i];

         double absoluteChange = Math.abs(nextValue - previousValue);
         if (absoluteChange > maxChange) maxChange = absoluteChange;
         previousValue = nextValue;
      }

      return maxChange;
   }
   
   public static boolean isContinuous(double[] values, double maxAllowedChange)
   {
      double maxChange = getMaximumAbsoluteChangeBetweenTicks(values);
      return (maxChange < maxAllowedChange);
   }
}

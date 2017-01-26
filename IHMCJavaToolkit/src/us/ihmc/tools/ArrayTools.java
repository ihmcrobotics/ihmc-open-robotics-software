package us.ihmc.tools;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.StringTokenizer;

public class ArrayTools
{

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

      StringTokenizer tokenizer = new StringTokenizer(line.replace("{", "").replace("[", "").replace("}", "").replace("]", "").replace(" ", "").replace(System.getProperty("line.separator"),
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
    * Use JUnitTools.assertArraysEquals
    */
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
    * Use JUnitTools.assertArraysEquals
    */
   public static boolean deltaEquals(float[] a, float[] b, float delta)
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

   public static int getIndexOfMaximumAbsoluteChangeBetweenTicks(double[] values)
   {
      int length = values.length;
      double maxChange = 0.0;
      int indexOfMaxChange = -1;

      double previousValue = values[0];

      for (int i = 1; i < length; i++)
      {
         double nextValue = values[i];

         double absoluteChange = Math.abs(nextValue - previousValue);
         if (absoluteChange > maxChange)
         {
            maxChange = absoluteChange;
            indexOfMaxChange = i;
         }
         previousValue = nextValue;
      }

      return indexOfMaxChange;
   }

   public static double getMaximumAbsoluteChangeBetweenTicks(double[] values)
   {
      int length = values.length;
      double maxChange = 0.0;

      double previousValue = values[0];
      for (int i = 1; i < length; i++)
      {
         double nextValue = values[i];

         double absoluteChange = Math.abs(nextValue - previousValue);
         if (absoluteChange > maxChange)
            maxChange = absoluteChange;
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

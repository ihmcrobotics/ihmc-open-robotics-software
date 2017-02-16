package us.ihmc.tools.io.xml;

import java.util.StringTokenizer;

import javax.vecmath.Color3f;
import javax.vecmath.Matrix3d;

/**
 * TODO This class needs be replaced by a proper XML library. @dcalvert
 */
public class XMLReaderUtility
{
   public static int getEndIndexOfSubString(int start, String fullString, String target)
   {
      return findSubstring(start, fullString, target)[1];
   }

   public static int getStartIndexOfSubString(int start, String fullString, String target)
   {
      return findSubstring(start, fullString, target)[0];
   }

   public static int[] findSubstring(int start, String fullString, String target)
   {
      // System.out.println(target);
      // System.out.println(fullString);
      int output[] = {-1, -1};

      if (start < 0)
      {
         return output;
      }

      if (fullString == null)
      {
         System.err.println("Full string is null!");
      }

      if (target == null)
      {
         System.err.println("Target is null!");
      }

      for (int i = start; target.length() + i <= fullString.length(); i++)
      {
         if (fullString.substring(i, target.length() + i).equalsIgnoreCase(target))
         {
            output[0] = i;
            output[1] = i + target.length();

            return output;
         }
      }

      return output;
   }

   public static void displayErrorMessage()
   {
      displayErrorMessage("Data Corrupted");
   }

   public static void displayErrorMessage(String message)
   {
      System.err.println(message);
   }

   public static String getMiddleString(int start, String fullString, String beginString, String endString)
   {
      int beginningIndex = findSubstring(start, fullString, beginString)[1];
      int endingIndex = findSubstring(beginningIndex, fullString, endString)[0];

      if ((beginningIndex == -1) || (endingIndex == -1))
      {
         return null;
      }
      else
      {
         return fullString.substring(beginningIndex, endingIndex).trim();
      }
   }

   public static double parseDoubleBetweenTwoStrings(int start, String fullString, String beginString, String endString)
   {
      return parseDouble(getMiddleString(start, fullString, beginString, endString));
   }

   public static boolean parseBooleanBetweenTwoStrings(int start, String fullString, String beginString, String endString)
   {
      return parseBoolean(getMiddleString(start, fullString, beginString, endString));
   }

   public static int parseIntegerBetweenTwoStrings(int start, String fullString, String beginString, String endString)
   {
      return parseInt(getMiddleString(start, fullString, beginString, endString));
   }

   public static boolean parseBoolean(String bool)
   {
      try
      {
         return Boolean.parseBoolean(bool);
      }
      catch (Exception e)
      {
         displayErrorMessage();

         return false;
      }
   }

   public static double parseDouble(String doubleValue)
   {
      try
      {
         return Double.parseDouble(doubleValue);
      }
      catch (Exception e)
      {
         displayErrorMessage();

         return -1;
      }
   }

   public static int parseInt(String integer)
   {
      try
      {
         return Integer.parseInt(integer);
      }
      catch (Exception e)
      {
         displayErrorMessage();

         return -1;
      }
   }

   public static javax.vecmath.Vector3d parseVector3d(String vector)
   {
      if (vector == null)
         return null;
      vector = replaceAll(vector, "(", "");
      vector = replaceAll(vector, ")", "");
      vector = vector.replaceAll(" ", "");
      StringTokenizer tokenizer = new StringTokenizer(vector, ",");
      try
      {
         return new javax.vecmath.Vector3d(new double[] {Double.parseDouble(tokenizer.nextToken()), Double.parseDouble(tokenizer.nextToken()),
                 Double.parseDouble(tokenizer.nextToken())});
      }
      catch (Exception e)
      {
         displayErrorMessage(vector);

         return null;
      }
   }

   public static String matrix3DToString(Matrix3d matrix3D)
   {
      String ret = matrix3D.toString().replaceAll("\n", ", ");

      return ret.substring(0, ret.length() - 2);
   }

   public static Matrix3d parseMatrix3d(String matrix3d)
   {
      matrix3d = matrix3d.replaceAll(" ", "");
      StringTokenizer tokenizer = new StringTokenizer(matrix3d, ",");
      try
      {
         Matrix3d m = new Matrix3d(Double.parseDouble(tokenizer.nextToken()), Double.parseDouble(tokenizer.nextToken()),
                                   Double.parseDouble(tokenizer.nextToken()), Double.parseDouble(tokenizer.nextToken()),
                                   Double.parseDouble(tokenizer.nextToken()), Double.parseDouble(tokenizer.nextToken()),
                                   Double.parseDouble(tokenizer.nextToken()), Double.parseDouble(tokenizer.nextToken()),
                                   Double.parseDouble(tokenizer.nextToken()));

         return m;
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      return null;
   }

   public static Color3f parseColor3f(String color)
   {
      try
      {
         StringTokenizer s = new StringTokenizer(color, ",");

         return new Color3f(new float[] {Float.parseFloat(s.nextToken()), Float.parseFloat(s.nextToken()), Float.parseFloat(s.nextToken())});
      }
      catch (Exception e)
      {
         return null;
      }
   }

   public static String replaceAll(String source, String toReplace, String replacement)
   {
      int idx = source.lastIndexOf(toReplace);
      if (idx != -1)
      {
         StringBuffer ret = new StringBuffer(source);
         ret.replace(idx, idx + toReplace.length(), replacement);

         while ((idx = source.lastIndexOf(toReplace, idx - 1)) != -1)
         {
            ret.replace(idx, idx + toReplace.length(), replacement);
         }

         source = ret.toString();
      }

      return source;
   }
}

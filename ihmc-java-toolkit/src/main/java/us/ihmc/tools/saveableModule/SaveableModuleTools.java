package us.ihmc.tools.saveableModule;

import java.io.Closeable;
import java.io.File;
import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.Scanner;

public class SaveableModuleTools
{
   static File ensureFileExists(File file)
   {
      try
      {
         file.getParentFile().mkdirs();
         file.createNewFile();
         return file;
      }
      catch (IOException e)
      {
         System.out.println(file.getAbsolutePath());
         e.printStackTrace();
         return null;
      }
   }

   static void closeStreamSilently(Closeable streamToClose)
   {
      try
      {
         if (streamToClose != null)
            streamToClose.close();
      }
      catch (Exception e)
      {
      }
   }

   public static double readNextDouble(Scanner scanner)
   {
      return readNextDouble(scanner, Double.NaN);
   }

   public static double readNextDouble(Scanner scanner, double defaultValue)
   {
      try
      {
         while (!scanner.hasNextDouble())
            scanner.next();
         return scanner.nextDouble();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }

   public static int readNextInt(Scanner scanner)
   {
      return readNextInt(scanner, -1);
   }

   public static int readNextInt(Scanner scanner, int defaultValue)
   {
      try
      {
         while (!scanner.hasNextInt())
            scanner.next();
         return scanner.nextInt();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }

   public static boolean readNextBoolean(Scanner scanner)
   {
      return readNextBoolean(scanner, false);
   }

   public static boolean readNextBoolean(Scanner scanner, boolean defaultValue)
   {
      try
      {
         while (!scanner.hasNextBoolean())
            scanner.next();
         return scanner.nextBoolean();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }
}

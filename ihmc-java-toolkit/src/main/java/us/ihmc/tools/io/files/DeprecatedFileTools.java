package us.ihmc.tools.io.files;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.ArrayList;

public class DeprecatedFileTools
{

   /**
    * @deprecated Use FileTools.newBufferedReader(path). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    */
   public static BufferedReader getFileReader(String filename) throws FileNotFoundException
   {
      return new BufferedReader(new InputStreamReader(new FileInputStream(filename)));
   }

   /**
    * @deprecated Use FileTools.newPrintWriter(path). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    */
   public static PrintWriter getFileWriter(String filename) throws FileNotFoundException, IOException
   {
      return new PrintWriter(new BufferedWriter(new FileWriter(filename)));
   }

   /**
    * @deprecated Use Use FileTools.newPrintWriter(path, true). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    */
   public static PrintWriter getFileWriterWithAppend(String filename) throws FileNotFoundException, IOException
   {
      return new PrintWriter(new BufferedWriter(new FileWriter(filename, true)));
   }

   /**
    * @deprecated Use FileTools.getFileDataOutputStream(path). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    */
   public static DataOutputStream getFileDataOutputStream(String filename) throws FileNotFoundException, IOException
   {
      return new DataOutputStream(new BufferedOutputStream(new FileOutputStream(filename)));
   }

   /**
    * @deprecated Use FileTools.getFileDataInputStream(path). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    */
   public static DataInputStream getFileDataInputStream(String filename) throws FileNotFoundException, IOException
   {
      return new DataInputStream(new BufferedInputStream(new FileInputStream(filename)));
   }

   /**
    * Gets all files in the directory
    * through recursive calls
    * 
    * @deprecated Use PathTools.walkRecursively(directory, basicFileVisitor). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    *
    * @param directory File
    * @return ArrayList
    */
   public static ArrayList<File> getAllFilesInDirectoryRecursive(File directory)
   {      
      return DeprecatedFileTools.getAllFilesInDirectoryRecursiveRegex(".*", directory);
   }

   /**
    * @deprecated Use PathTools.findAllPathsRecursivelyThatMatchRegex(rootPath, regex). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    */
   public static ArrayList<File> getAllFilesInDirectoryRecursiveRegex(String regex, File directory)
   {
   // File[] files = new File[0];
      if (!directory.isDirectory())
      {
         throw new IllegalArgumentException(directory.getAbsolutePath() + " is not a directory");
      }
   
      ArrayList<File> ret = new ArrayList<File>();
      File[] contents = directory.listFiles();
      for (File file : contents)
      {
         if (!file.isHidden())
         {
            if (file.isDirectory())
            {
               ret.addAll(getAllFilesInDirectoryRecursiveRegex(regex, file));
            }
            else
            {
               if (file.getName().matches(regex))
                  ret.add(file);
            }
         }
      }
   
      return ret;
   }

   /**
    * @deprecated Use PathTools.findAllPathsRecursivelyThatMatchRegex(rootPath, regex). See <a href="http://docs.oracle.com/javase/tutorial/essential/io/fileio.html">File I/O (Featuring NIO.2)</a>
    */
   public static ArrayList<File> getAllFilesInDirectoryWithSuffix(String suffix, File directory)
   {
      if (directory.getName().equals(""))
         directory = new File(".");
   
      if (!directory.isDirectory())
      {
         throw new RuntimeException("This method can only be called with a directory: " + directory.toString());
      }
   
      ArrayList<File> allValidFiles = new ArrayList<File>();
   
      File[] allFiles = directory.listFiles();
      if ((allFiles == null) || (allFiles.length <= 0))
         return allValidFiles;
   
      for (File file : allFiles)
      {
         if (file.getName().endsWith(suffix) &&!file.isDirectory())
            allValidFiles.add(file);
      }
   
      return allValidFiles;
   }

}

package us.ihmc.tools.io.files;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.io.Writer;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.tools.UnitConversions;

public class FileTools
{
   private static final String RESOURCES_FOLDER_NAME = "resources";
   
   public static Path deriveResourcesPath(Class<?> clazz)
   {
      List<String> pathNames = new ArrayList<String>();
      
      String[] packageNames = clazz.getPackage().getName().split("\\.");
      
      pathNames.addAll(Arrays.asList(packageNames));
      pathNames.add(StringUtils.uncapitalize(clazz.getSimpleName()));
      
      return Paths.get(RESOURCES_FOLDER_NAME, pathNames.toArray(new String[0]));
   }
   
   public static List<String> readAllLines(Path path)
   {
      try
      {
         return Files.readAllLines(path, Charset.forName("UTF-8"));
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }
   
   public static void ensureDirectoryExists(Path path)
   {
      try
      {
         if (!Files.exists(path.getParent()))
            ensureDirectoryExists(path.getParent());
         
         if (!Files.exists(path))
            Files.createDirectory(path);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public static PrintWriter newPrintWriter(Path path)
   {
      return newPrintWriter(path, false);
   }
   
   public static void write(Path path, byte[] bytes, OpenOption... options)
   {
      try
      {
         Files.write(path, bytes, options);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public static BufferedReader newBufferedReader(Path path) throws IOException
   {
      return Files.newBufferedReader(path, Charset.defaultCharset());
   }
   
   public static PrintWriter newPrintWriter(Path path, boolean append)
   {
      try
      {
         Writer outWriter = new FileWriter(path.toFile(), append);
         BufferedWriter bufferedWriter = new BufferedWriter(outWriter);
         return new PrintWriter(bufferedWriter); 
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return null;
      }
   }
   
   public static boolean checkIfSerializable(Object objectToTest)
   {
      ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
      try
      {
         ObjectOutputStream testStream = new ObjectOutputStream(byteArrayOutputStream);
         testStream.writeObject(objectToTest);
         testStream.flush();
         return true;
      } 
      catch (IOException e1)
      {
         e1.printStackTrace();
         return false;
      }
   }
   
   public static void concatenateFilesTogether(List<Path> filesToConcatenate, Path concatenatedFile)
   {
      DataOutputStream dataOutputStream = getFileDataOutputStream(concatenatedFile);

      try
      {
         for (Path fileToConcatenate : filesToConcatenate)
         {
            DataInputStream dataInputStream = getFileDataInputStream(fileToConcatenate);

            while (dataInputStream.available() > 0)
            {
               dataOutputStream.write(dataInputStream.read());
            }

            dataInputStream.close();
         }

         dataOutputStream.flush();
         dataOutputStream.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public static DataOutputStream getFileDataOutputStream(Path file)
   {
      return getFileDataOutputStream(file, UnitConversions.kibibytesToBytes(8));
   }
   
   public static DataOutputStream getFileDataOutputStream(Path file, int bufferSize)
   {
      try
      {
         return new DataOutputStream(new BufferedOutputStream(new FileOutputStream(file.toFile()), bufferSize));
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         return null;
      }
   }
   
   public static DataInputStream getFileDataInputStream(Path file)
   {
      return getFileDataInputStream(file, UnitConversions.kibibytesToBytes(8));
   }
   
   public static DataInputStream getFileDataInputStream(Path file, int bufferSize)
   {
      try
      {
         return new DataInputStream(new BufferedInputStream(new FileInputStream(file.toFile()), bufferSize));
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         return null;
      }
   }
   
   public static Path getTemporaryDirectoryPath()
   {
      return Paths.get(System.getProperty("java.io.tmpdir"));
   }
   
   // START DEPRECATED METHODS HERE

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
      return getAllFilesInDirectoryRecursiveRegex(".*", directory);
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

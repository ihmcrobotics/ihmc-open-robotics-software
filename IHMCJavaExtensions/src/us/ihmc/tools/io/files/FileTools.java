package us.ihmc.tools.io.files;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.io.FileUtils;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.WriteOption;

/**
 * A collection of tools to bridge the NIO.2 and Apache Commons IO APIs.
 * Includes some convenient tools that don't force the use of try-catch.
 */
public class FileTools
{
   /** A carriage return */
   private static final byte CARRIAGE_RETURN = '\r';
   
   /** A newline */
   private static final byte NEWLINE = '\n';
   
   /**
    * Delete a file or directory quietly. A bridge from Java's NIO.2 to Apache Commons IO.
    * 
    * @see {@link FileUtils#deleteQuietly(File)}
    * 
    * @param path file or directory to be deleted
    */
   public static void deleteQuietly(Path path)
   {
      FileUtils.deleteQuietly(path.toFile());
   }

   /**
    * Reads all lines from a file. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link Files#readAllLines()}
    * 
    * @param path file to read lines from
    * @param exceptionHandler default exception handler
    * @return list of strings
    */
   @SuppressWarnings("unchecked")
   public static List<String> readAllLines(Path path, DefaultExceptionHandler exceptionHandler)
   {
      try
      {
         return Files.readAllLines(path);
      }
      catch (IOException ioException)
      {
         return (List<String>) exceptionHandler.handleException(ioException);
      }
   }

   /**
    * Write a list of Strings to lines in a file. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @param lines lines to be written
    * @param path file
    * @param writeOption append or truncate
    * @param defaultExceptionHandler default exception handler
    */
   public static void writeAllLines(List<String> lines, Path path, WriteOption writeOption, DefaultExceptionHandler defaultExceptionHandler)
   {
      PrintWriter printer = newPrintWriter(path, writeOption, defaultExceptionHandler);
      
      for (String line : lines)
      {
         printer.println(line);
      }
      
      printer.close();
   }

   /**
    * Creates a new PrintWriter. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @param path file to open
    * @param writeOption truncate or append
    * @param defaultExceptionHandler default exception handler
    * @return new print writer
    */
   public static PrintWriter newPrintWriter(Path path, WriteOption writeOption, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         return newPrintWriter(path, writeOption);
      }
      catch (IOException ioException)
      {
         return (PrintWriter) defaultExceptionHandler.handleException(ioException);
      }
   }
   /**
    * Creates a new PrintWriter. Uses Java's NIO.2 API.
    * 
    * @see {@link Files#newBufferedWriter(Path)}
    * 
    * @param path file to open
    * @param writeOption append or truncate
    * @return new print writer
    * @throws IOException 
    */
   public static PrintWriter newPrintWriter(Path path, WriteOption writeOption) throws IOException
   {
      if (writeOption == WriteOption.APPEND)
      {
         return new PrintWriter(Files.newBufferedWriter(path, StandardOpenOption.APPEND));
      }
      else // (writeOption == WriteOption.TRUNCATE)
      {
         return new PrintWriter(Files.newBufferedWriter(path));
      }
   }

   public static List<String> readLinesFromBytes(byte[] bytes)
   {
      List<String> lines = new ArrayList<>();
      try(BufferedReader reader = new BufferedReader(new InputStreamReader(new ByteArrayInputStream(bytes), Charset.forName("UTF-8").newDecoder())))
      {
         while(true)
         {
            String line = reader.readLine();
            if (line != null)
               lines.add(line);
            else
               break;
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      return lines;
   }
   
   public static byte[] replaceLineInFile(int lineIndex, String newLine, byte[] fileBytes, List<String> fileLines)
   {
      byte[] newBytes = new byte[fileBytes.length - fileLines.get(lineIndex).length() + newLine.length()];
      
      int newBytesIndex = 0;
      int fileBytesIndex = 0;
      for (int fileLineIndex = 0; fileLineIndex < fileLines.size(); fileLineIndex++)
      {
         if (fileLineIndex == lineIndex)
         {
            for (byte b : newLine.getBytes())
            {
               newBytes[newBytesIndex++] = b;
            }
            
            fileBytesIndex += fileLines.get(fileLineIndex).length();
            fileLines.set(fileLineIndex, newLine);
         }
         else
         {
            for (byte b : fileLines.get(fileLineIndex).getBytes())
            {
               newBytes[newBytesIndex++] = b;
               ++fileBytesIndex;
            }
         }
         
         if (fileBytes.length > fileBytesIndex + 1 &&
             fileBytes[fileBytesIndex] == CARRIAGE_RETURN && fileBytes[fileBytesIndex + 1] == NEWLINE)
         {
            newBytes[newBytesIndex++] = fileBytes[fileBytesIndex++];
            newBytes[newBytesIndex++] = fileBytes[fileBytesIndex++];
         }
         else if (fileBytesIndex < fileBytes.length && (fileBytes[fileBytesIndex] == CARRIAGE_RETURN || fileBytes[fileBytesIndex] == NEWLINE))
         {
            newBytes[newBytesIndex++] = fileBytes[fileBytesIndex++];
         }
      }
      
      return newBytes;
   }
   
   public static byte[] readAllBytes(Path path)
   {
      try
      {
         return Files.readAllBytes(path);
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
   
   public static boolean checkIfSerializable(Object objectToTest)
   {
      try(ObjectOutputStream testStream = new ObjectOutputStream(new ByteArrayOutputStream()))
      {
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
      try(DataOutputStream dataOutputStream = getFileDataOutputStream(concatenatedFile))
      {
         for (Path fileToConcatenate : filesToConcatenate)
         {
            try(DataInputStream dataInputStream = getFileDataInputStream(fileToConcatenate))
            {
               while (dataInputStream.available() > 0)
               {
                  dataOutputStream.write(dataInputStream.read());
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }

         dataOutputStream.flush();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   public static DataOutputStream getFileDataOutputStream(Path file)
   {
      return getFileDataOutputStream(file, Conversions.kibibytesToBytes(8));
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
      return getFileDataInputStream(file, Conversions.kibibytesToBytes(8));
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
}

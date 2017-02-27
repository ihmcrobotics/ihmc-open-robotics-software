package us.ihmc.commons.nio;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.io.FileUtils;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;

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
    * @see {@link java.nio.file.Files#readAllLines(Path)}
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
    * Reads all lines from a file. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link java.nio.file.Files#readAllLines(Path, Charset)}
    * 
    * @param path file to read lines from
    * @param charset character set to use
    * @param exceptionHandler default exception handler
    * @return list of strings
    */
   @SuppressWarnings("unchecked")
   public static List<String> readAllLines(Path path, Charset charset, DefaultExceptionHandler exceptionHandler)
   {
      try
      {
         return Files.readAllLines(path, charset);
      }
      catch (IOException ioException)
      {
         return (List<String>) exceptionHandler.handleException(ioException);
      }
   }

   /**
    * Reads all the bytes from a file. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link java.nio.file.Files#readAllBytes(Path)}
    * 
    * @param path file to read lines from
    * @param exceptionHandler default exception handler
    * @return File as a byte array.
    */
   public static byte[] readAllBytes(Path path, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         return Files.readAllBytes(path);
      }
      catch (IOException ioException)
      {
         return (byte[]) defaultExceptionHandler.handleException(ioException);
      }
   }

   /**
    * Writes bytes to a file. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link java.nio.file.Files#write(Path, byte[], OpenOption...)}
    * 
    * @param path file to write to
    * @param bytes bytes to write
    * @param writeOption append or truncate
    * @param exceptionHandler default exception handler
    */
   public static void write(Path path, byte[] bytes, WriteOption writeOption, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         Files.write(path, bytes, writeOption.getOptions());
      }
      catch (IOException ioException)
      {
         defaultExceptionHandler.handleException(ioException);
      }
   }

   /**
    * Write lines of text to a file. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link java.nio.file.Files#write(Path, Iterable, OpenOption...)}
    * 
    * @param path file to write to
    * @param lines lines to write
    * @param writeOption append or truncate
    * @param exceptionHandler default exception handler
    */
   public static void write(Path path, Iterable<? extends CharSequence> lines, WriteOption writeOption, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         Files.write(path, lines, writeOption.getOptions());
      }
      catch (IOException ioException)
      {
         defaultExceptionHandler.handleException(ioException);
      }
   }

   /**
    * Write lines of text to a file. Uses Java's NIO.2 API.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link java.nio.file.Files#write(Path, Iterable, Charset, OpenOption...)}
    * 
    * @param path file to write to
    * @param lines lines to write
    * @param charset charset to use
    * @param writeOption append or truncate
    * @param exceptionHandler default exception handler
    */
   public static void write(Path path, Iterable<? extends CharSequence> lines, Charset charset, WriteOption writeOption,
                            DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         Files.write(path, lines, charset, writeOption.getOptions());
      }
      catch (IOException ioException)
      {
         defaultExceptionHandler.handleException(ioException);
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
      lines.forEach(line -> printer.println(line));
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
      return new PrintWriter(Files.newBufferedWriter(path, writeOption.getOptions()));
   }

   /**
    * Read bytes into a list of strings using {@link BufferedReader#readLine()}.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @param bytes bytes to read
    * @param defaultExceptionHandler default exception handler
    * @return list of strings
    */
   public static List<String> readLinesFromBytes(byte[] bytes, DefaultExceptionHandler defaultExceptionHandler)
   {
      List<String> lines = new ArrayList<>();
      try (BufferedReader reader = new BufferedReader(new InputStreamReader(new ByteArrayInputStream(bytes), StandardCharsets.UTF_8.newDecoder())))
      {
         while (true)
         {
            String line = reader.readLine();
            if (line != null)
            {
               lines.add(line);
            }
            else
            {
               break;
            }
         }
      }
      catch (IOException ioException)
      {
         defaultExceptionHandler.handleException(ioException);
      }
      return lines;
   }

   /**
    * Replace a line in a file by index with a replacement line. For efficiency, it is required
    * to pass in the file as an array of bytes and also as a list of strings by line.
    * 
    * @param lineIndex line number starting at 0 of line to replace
    * @param newLine replacement line
    * @param fileBytes file as an array of bytes
    * @param fileLines file as a list of lines
    * @return updated file with replacement line as an array of bytes
    */
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

         if (fileBytes.length > fileBytesIndex + 1 && fileBytes[fileBytesIndex] == CARRIAGE_RETURN && fileBytes[fileBytesIndex + 1] == NEWLINE)
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

   /**
    * <p>Ensure directory exists. Performs the following:</p>
    * 
    * <li>Recursively perform this on parents.</li>
    * <li>If <code>path</code> is a file, delete and make new directory.</li>
    * <li>If <code>path</code> does not exist, create directory.</li>
    * 
    * @param path path of directory to ensure existence of
    * @throws IOException 
    */
   public static void ensureDirectoryExists(Path path) throws IOException
   {
      if (path.getParent() != null && !Files.exists(path.getParent()))
      {
         ensureDirectoryExists(path.getParent());
      }

      if (Files.exists(path) && !Files.isDirectory(path))
      {
         FileTools.deleteQuietly(path);
         Files.createDirectory(path);
      }

      if (!Files.exists(path))
      {
         Files.createDirectory(path);
      }
   }

   /**
    * <p>Ensure directory exists. Performs the following:</p>
    * 
    * <li>Recursively perform this on parents.</li>
    * <li>If <code>path</code> is a file, delete and make new directory.</li>
    * <li>If <code>path</code> does not exist, create directory.</li>
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @param path path of directory to ensure existence of
    * @param defaultExceptionHandler default exception handler
    */
   public static void ensureDirectoryExists(Path path, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         ensureDirectoryExists(path);
      }
      catch (IOException ioException)
      {
         defaultExceptionHandler.handleException(ioException);
      }
   }

   /**
    * <p>Ensure file exists. Performs the following:</p>
    * 
    * <li>Recursively perform this on parents.</li>
    * <li>If <code>path</code> is a directory, delete and create new file.</li>
    * <li>If <code>path</code> does not exist, create file.</li>
    * 
    * @param path path of file to ensure existence of
    * @throws IOException 
    */
   public static void ensureFileExists(Path path) throws IOException
   {
      if (path.getParent() != null && !Files.exists(path.getParent()))
      {
         ensureDirectoryExists(path.getParent());
      }

      if (Files.exists(path) && Files.isDirectory(path))
      {
         FileTools.deleteQuietly(path);
         Files.createFile(path);
      }

      if (!Files.exists(path))
      {
         Files.createFile(path);
      }
   }

   /**
    * <p>Ensure file exists. Performs the following:</p>
    * 
    * <li>Recursively perform this on parents.</li>
    * <li>If <code>path</code> is a directory, delete and create new file.</li>
    * <li>If <code>path</code> does not exist, create file.</li>
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @param path path of file to ensure existence of
    */
   public static void ensureFileExists(Path path, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         ensureFileExists(path);
      }
      catch (IOException ioException)
      {
         defaultExceptionHandler.handleException(ioException);
      }
   }

   /**
    * Concatenate N files together into one file.
    * 
    * @param filesToConcatenate files to concatenate
    * @param concatenatedFile concatenated file
    * @param defaultExceptionHandler default exception handler
    * @throws IOException 
    */
   public static void concatenateFiles(List<Path> filesToConcatenate, Path concatenatedFile) throws IOException
   {
      DataOutputStream concatenatedFileOutputStream = newFileDataOutputStream(concatenatedFile);

      for (Path fileToConcatenate : filesToConcatenate)
      {
         DataInputStream fileToConcatenateInputStream = newFileDataInputStream(fileToConcatenate);

         while (fileToConcatenateInputStream.available() > 0)
         {
            concatenatedFileOutputStream.write(fileToConcatenateInputStream.read());
         }
      }

      concatenatedFileOutputStream.flush();
   }

   /**
    * Concatenate N files together into one file.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @param filesToConcatenate files to concatenate
    * @param concatenatedFile concatenated file
    * @param defaultExceptionHandler default exception handler
    * @throws IOException 
    */
   public static void concatenateFiles(List<Path> filesToConcatenate, Path concatenatedFile, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         concatenateFiles(filesToConcatenate, concatenatedFile);
      }
      catch (IOException ioException)
      {
         defaultExceptionHandler.handleException(ioException);
      }
   }

   /**
    * Creates a new data output stream to a file for writing.
    * 
    * @see {@link DataOutputStream}, {@link BufferedOutputStream}, {@link FileOutputStream}.
    * 
    * @param file file to create stream from
    * @return dataOutputStream file data output stream
    * @throws FileNotFoundException
    */
   public static DataOutputStream newFileDataOutputStream(Path file) throws FileNotFoundException
   {
      return newFileDataOutputStream(file, Conversions.kibibytesToBytes(8));
   }

   /**
    * Creates a new data output stream to a file for writing.
    * 
    * @see {@link DataOutputStream}, {@link BufferedOutputStream}, {@link FileOutputStream}.
    * 
    * @param file file to create stream from
    * @param bufferSizeInBytes buffer size in bytes
    * @return dataOutputStream file data output stream
    * @throws FileNotFoundException
    */
   public static DataOutputStream newFileDataOutputStream(Path file, int bufferSizeInBytes) throws FileNotFoundException
   {
      return new DataOutputStream(new BufferedOutputStream(new FileOutputStream(file.toFile()), bufferSizeInBytes));
   }

   /**
    * Creates a new data output stream to a file for writing.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link DataOutputStream}, {@link BufferedOutputStream}, {@link FileOutputStream}.
    * 
    * @param file file to create stream from
    * @param defaultExceptionHandler default exception handler
    * @return dataOutputStream file data output stream
    * @throws FileNotFoundException
    */
   public static DataOutputStream newFileDataOutputStream(Path file, DefaultExceptionHandler defaultExceptionHandler)
   {
      return newFileDataOutputStream(file, Conversions.kibibytesToBytes(8), defaultExceptionHandler);
   }

   /**
    * Creates a new data output stream to a file for writing.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link DataOutputStream}, {@link BufferedOutputStream}, {@link FileOutputStream}.
    * 
    * @param file file to create stream from
    * @param bufferSizeInBytes buffer size in bytes
    * @param defaultExceptionHandler default exception handler
    * @return dataOutputStream file data output stream
    * @throws FileNotFoundException
    */
   public static DataOutputStream newFileDataOutputStream(Path file, int bufferSizeInBytes, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         return newFileDataOutputStream(file, bufferSizeInBytes);
      }
      catch (FileNotFoundException fileNotFoundException)
      {
         return (DataOutputStream) defaultExceptionHandler.handleException(fileNotFoundException);
      }
   }

   /**
    * Creates a new data input stream to read from a file.
    * 
    * @see {@link DataInputStream}, {@link BufferedInputStream}, {@link FileInputStream}.
    * 
    * @param file file to create stream from
    * @return dataInputStream file data input stream
    * @throws FileNotFoundException
    */
   public static DataInputStream newFileDataInputStream(Path file) throws FileNotFoundException
   {
      return newFileDataInputStream(file, Conversions.kibibytesToBytes(8));
   }

   /**
    * Creates a new data input stream to read from a file.
    * 
    * @see {@link DataInputStream}, {@link BufferedInputStream}, {@link FileInputStream}.
    * 
    * @param file file to create stream from
    * @param bufferSizeInBytes buffer size in bytes
    * @return dataInputStream file data input stream
    * @throws FileNotFoundException
    */
   public static DataInputStream newFileDataInputStream(Path file, int bufferSizeInBytes) throws FileNotFoundException
   {
      return new DataInputStream(new BufferedInputStream(new FileInputStream(file.toFile()), bufferSizeInBytes));
   }

   /**
    * Creates a new data input stream to read from a file.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link DataInputStream}, {@link BufferedInputStream}, {@link FileInputStream}.
    * 
    * @param file file to create stream from
    * @param defaultExceptionHandler default exception handler
    * @return dataInputStream file data input stream
    * @throws FileNotFoundException
    */
   public static DataInputStream newFileDataInputStream(Path file, DefaultExceptionHandler defaultExceptionHandler)
   {
      return newFileDataInputStream(file, Conversions.kibibytesToBytes(8), defaultExceptionHandler);
   }

   /**
    * Creates a new data input stream to read from a file.
    * 
    * <p>WARNING: For use only when there is no meaningful way to handle failure.</p>
    * 
    * @see {@link DataInputStream}, {@link BufferedInputStream}, {@link FileInputStream}.
    * 
    * @param file file to create stream from
    * @param bufferSizeInBytes buffer size in bytes
    * @param defaultExceptionHandler default exception handler
    * @return dataInputStream file data input stream
    * @throws FileNotFoundException
    */
   public static DataInputStream newFileDataInputStream(Path file, int bufferSizeInBytes, DefaultExceptionHandler defaultExceptionHandler)
   {
      try
      {
         return newFileDataInputStream(file, bufferSizeInBytes);
      }
      catch (FileNotFoundException fileNotFoundException)
      {
         return (DataInputStream) defaultExceptionHandler.handleException(fileNotFoundException);
      }
   }
}

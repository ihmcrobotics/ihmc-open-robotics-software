package us.ihmc.tools.io.files;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
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
import java.util.List;

import org.apache.commons.io.FileUtils;

import us.ihmc.commons.Conversions;

public class FileTools
{
   private static final byte CARRIAGE_RETURN = '\r';
   private static final byte NEWLINE = '\n';
   
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
   
   public static void writeAllLines(List<String> lines, Path path)
   {
      PrintWriter printer = newPrintWriter(path);
      
      for (String line : lines)
      {
         printer.println(line);
      }
      
      printer.close();
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
   
   public static void deleteDirectory(Path directoryPath)
   {
      FileUtils.deleteQuietly(directoryPath.toFile());
   }
}

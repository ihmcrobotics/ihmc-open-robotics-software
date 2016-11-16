package us.ihmc.robotDataLogger.compressor;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import org.tukaani.xz.XZInputStream;

import com.google.common.hash.Hashing;
import com.google.common.io.Files;

import us.ihmc.robotDataLogger.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.LogProperties;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.tools.compression.SnappyUtils;

public class LogFileDecompressor extends YoVariableLogReader
{
   public static final String out = "/home/jesper/scratch/decompressed/";
   private int bufferedElements;

   private FileChannel outputChannel;
   private FileChannel indexChannel;

   private final ByteBuffer indexLine = ByteBuffer.allocateDirect(16);

   private int totalElements = 0;

   public LogFileDecompressor(File compressedDirectory, LogProperties logProperties) throws IOException
   {
      super(compressedDirectory, logProperties);

      long startTime = System.nanoTime();
      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();
      int numberOfVariables = YoVariableHandshakeParser.getNumberOfVariables(handshakeData);

      CompressionProperties properties = new CompressionProperties();
      FileInputStream propertiesFile = new FileInputStream(new File(compressedDirectory, "robotData.compressed"));
      properties.load(propertiesFile);
      propertiesFile.close();

      bufferedElements = properties.getNumberOfBufferedElements();

      System.out.println("Found " + numberOfVariables + " variables.");
      System.out.println("Reading " + bufferedElements + " data points at a time");

      FileOutputStream outputStream = new FileOutputStream(new File(out, logProperties.getVariableDataFile()));
      outputChannel = outputStream.getChannel();

      FileOutputStream indexStream = new FileOutputStream(new File(out, logProperties.getVariablesIndexFile()));
      indexChannel = indexStream.getChannel();

      byte[] data = new byte[bufferedElements * numberOfVariables * 8];
      for (int i = 0; i < properties.getCompressedDataFiles(); i++)
      {

         FileInputStream logInputStream = new FileInputStream(new File(compressedDirectory, "robotData." + i + ".xz"));
         XZInputStream xzInputStream = new XZInputStream(logInputStream);

         int read = xzInputStream.read(data, 0, data.length);

         int elements = read / (numberOfVariables * 8);

         System.out.println("Read " + read + " bytes, " + elements + " elements for " + i);
         totalElements += elements;
         writeDataToStream(data, elements, numberOfVariables);

         xzInputStream.close();
      }

      System.out.println("Read " + totalElements + " total");

      outputChannel.close();
      outputStream.close();

      indexChannel.close();
      indexStream.close();

      copyMetaData(new File(out));
      
      System.out.println("Decompression took " + TimeTools.nanoSecondstoSeconds(System.nanoTime() - startTime));
      checkChecksums(properties);
      
   }

   private void checkChecksums(CompressionProperties properties) throws IOException
   {
      System.out.println("Validating checksums.");
      File logdata = new File(out, logProperties.getVariableDataFile());
      if (!properties.getDataChecksum().equals(Files.hash(logdata, Hashing.sha1()).toString()))
      {
         System.err.println("Variable data does not match with pre-compression data. Expect an unloadable data file.");
         return;
      }

      File index = new File(out, logProperties.getVariablesIndexFile());
      if (!properties.getTimestampChecksum().equals(Files.hash(index, Hashing.sha1()).toString()))
      {
         System.err.println("Timestamp data does not match with pre-compression data. A bug has been found in the LogFileDecompressor and fixing it will make this data file readable again.");
      }
      System.out.println("Checksums validated.");
   }

   private void writeDataToStream(byte[] data, int elements, int numberOfVariables) throws IOException
   {
      ByteBuffer dataBuffer = ByteBuffer.wrap(data);

      ByteBuffer timeElement = ByteBuffer.allocate(numberOfVariables * 8);
      ByteBuffer compressed = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(numberOfVariables * 8));

      for (int e = 0; e < elements; e++)
      {
         timeElement.clear();
         compressed.clear();

         for (int i = 0; i < numberOfVariables; i++)
         {
            timeElement.put(dataBuffer.get((i * elements + e) * 8));
            timeElement.put(dataBuffer.get((i * elements + e) * 8 + 1));
            timeElement.put(dataBuffer.get((i * elements + e) * 8 + 2));
            timeElement.put(dataBuffer.get((i * elements + e) * 8 + 3));
            timeElement.put(dataBuffer.get((i * elements + e) * 8 + 4));
            timeElement.put(dataBuffer.get((i * elements + e) * 8 + 5));
            timeElement.put(dataBuffer.get((i * elements + e) * 8 + 6));
            timeElement.put(dataBuffer.get((i * elements + e) * 8 + 7));
         }

         timeElement.flip();
         long timestamp = timeElement.getLong(0);
         try
         {
            SnappyUtils.compress(timeElement, compressed);
         }
         catch (IOException e1)
         {
            throw new RuntimeException(e1);
         }
         compressed.flip();

         long offset = outputChannel.position();

         indexLine.clear();
         indexLine.putLong(timestamp);
         indexLine.putLong(offset);
         indexLine.flip();

         indexChannel.write(indexLine);
         outputChannel.write(compressed);

      }

   }

   public static void main(String[] args) throws IOException
   {
      //      File logDirectory = FileSelectionDialog.loadDirectoryWithFileNamed(YoVariableLoggerListener.propertyFile);
      File logDirectory = new File("/home/jesper/scratch/compressed/");
      if (logDirectory != null)
      {
         LogPropertiesReader logProperties = new LogPropertiesReader(new File(logDirectory, YoVariableLoggerListener.propertyFile));

         new LogFileDecompressor(logDirectory, logProperties);
      }
   }
}

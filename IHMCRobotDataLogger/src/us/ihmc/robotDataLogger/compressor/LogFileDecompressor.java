package us.ihmc.robotDataLogger.compressor;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.concurrent.atomic.AtomicInteger;

import org.tukaani.xz.XZInputStream;

import com.google.common.hash.Hashing;
import com.google.common.io.Files;

import us.ihmc.commons.Conversions;
import us.ihmc.robotDataLogger.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.logger.LogProperties;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataLogger.logger.util.CustomProgressMonitor;
import us.ihmc.robotDataLogger.logger.util.ProgressMonitorInterface;
import us.ihmc.tools.compression.SnappyUtils;

public class LogFileDecompressor extends YoVariableLogReader
{
   private final File targetFile;
   private int bufferedElements;
   
   private final PrintStream out;

   private FileChannel outputChannel;
   private FileChannel indexChannel;

   private final ByteBuffer indexLine = ByteBuffer.allocateDirect(16);

   private int totalElements = 0;
   
   private final ProgressMonitorInterface progressMonitor;
   private final AtomicInteger progress = new AtomicInteger();

   public LogFileDecompressor(File compressedDirectory, File targetFile, LogProperties logProperties, ProgressMonitorInterface progressMonitor) throws IOException
   {
      super(compressedDirectory, logProperties);
      this.targetFile = targetFile;
      
      
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

      this.progressMonitor = progressMonitor;
      progressMonitor.initialize("Decompressing data", null, 0, properties.getCompressedDataFiles() + 1);
      this.out = progressMonitor.getPrintStream();

      bufferedElements = properties.getNumberOfBufferedElements();

      out.println("Found " + numberOfVariables + " variables.");
      out.println("Reading " + bufferedElements + " data points at a time");

      FileOutputStream outputStream = new FileOutputStream(new File(targetFile, logProperties.getVariableDataFile()));
      outputChannel = outputStream.getChannel();

      FileOutputStream indexStream = new FileOutputStream(new File(targetFile, logProperties.getVariablesIndexFile()));
      indexChannel = indexStream.getChannel();

      byte[] data = new byte[bufferedElements * numberOfVariables * 8];
      for (int i = 0; i < properties.getCompressedDataFiles(); i++)
      {
         FileInputStream logInputStream = new FileInputStream(new File(compressedDirectory, "robotData." + i + ".xz"));
         XZInputStream xzInputStream = new XZInputStream(logInputStream);

         int read = xzInputStream.read(data, 0, data.length);

         int elements = read / (numberOfVariables * 8);

         out.println("Read " + read + " bytes, " + elements + " elements for " + i);
         totalElements += elements;
         writeDataToStream(data, elements, numberOfVariables);

         xzInputStream.close();
         
         incrementProgress();
      }

      out.println("Read " + totalElements + " total");

      outputChannel.close();
      outputStream.close();

      indexChannel.close();
      indexStream.close();

      copyMetaData(targetFile);
      
      out.println("Decompression took " + Conversions.nanoSecondstoSeconds(System.nanoTime() - startTime));
      checkChecksums(properties);
      incrementProgress();
      
      progressMonitor.close();
   }
   
   private void incrementProgress()
   {
      progressMonitor.setProgress(progress.incrementAndGet());
   }

   private void checkChecksums(CompressionProperties properties) throws IOException
   {
      out.println("Validating checksums.");
      File logdata = new File(targetFile, logProperties.getVariableDataFile());
      if (!properties.getDataChecksum().equals(Files.hash(logdata, Hashing.sha1()).toString()))
      {
         throw new IOException("Variable data does not match with pre-compression data. Expect an unloadable data file.");
      }

      File index = new File(targetFile, logProperties.getVariablesIndexFile());
      if (!properties.getTimestampChecksum().equals(Files.hash(index, Hashing.sha1()).toString()))
      {
         throw new IOException("Timestamp data does not match with pre-compression data. A bug has been found in the LogFileDecompressor and fixing it will make this data file readable again.");
      }
      out.println("Checksums validated.");
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
         SnappyUtils.compress(timeElement, compressed);
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
      File logDirectory = new File("/home/jesper/scratch/compressed/");
      File target = new File("/home/jesper/scratch/decompressed/");
      if (logDirectory != null)
      {
         LogPropertiesReader logProperties = new LogPropertiesReader(new File(logDirectory, YoVariableLoggerListener.propertyFile));
         CustomProgressMonitor customProgressMonitor = new CustomProgressMonitor();
         new LogFileDecompressor(logDirectory, target, logProperties, customProgressMonitor);
      }
   }
}

package us.ihmc.robotDataVisualizer.compressor;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.ByteBuffer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicInteger;

import org.tukaani.xz.LZMA2Options;
import org.tukaani.xz.XZOutputStream;

import com.google.common.hash.Hashing;
import com.google.common.io.Files;

import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.robotDataLogger.logger.YoVariableLogReader;
import us.ihmc.robotDataLogger.logger.YoVariableLoggerListener;
import us.ihmc.robotDataVisualizer.logger.util.CustomProgressMonitor;
import us.ihmc.robotDataVisualizer.logger.util.ProgressMonitorInterface;

public class LogFileCompressor extends YoVariableLogReader
{
   public static final int NUMBER_OF_THREADS = 8;
   public static final int MAX_BUFFER_SIZE_IN_BYTE = 1000000000;
   public static final int COMPRESSION_LEVEL = 3;

   private final PrintStream out;
   private final File targetDirectory;

   private int bufferedElements;
   private volatile int compressedDataFiles = 0;

   private final ExecutorService threadPool = Executors.newFixedThreadPool(NUMBER_OF_THREADS);
   private final LinkedBlockingQueue<DataBuffer> availableBuffers = new LinkedBlockingQueue<>();
   private final LinkedBlockingQueue<DataBuffer> writableBuffers = new LinkedBlockingQueue<>();

   private final AtomicInteger progress = new AtomicInteger(0);
   private final ProgressMonitorInterface progressMonitor;

   public LogFileCompressor(File logDirectory, File targetDirectory, LogProperties logProperties, ProgressMonitorInterface progressMonitor) throws IOException
   {
      super(logDirectory, logProperties);
      this.targetDirectory = targetDirectory;

      initialize();

      int maxBufferedElements = MAX_BUFFER_SIZE_IN_BYTE / 8;

      bufferedElements = maxBufferedElements / getNumberOfVariables();

      this.progressMonitor = progressMonitor;
      progressMonitor.initialize("Log file compression", null, 0, getNumberOfEntries() / bufferedElements);
      out = progressMonitor.getPrintStream();

      out.println("Reading " + getNumberOfEntries() + " entries.");
      out.println("Found " + getNumberOfVariables() + " variables.");
      out.println("Writing " + bufferedElements + " data points at a time");
      out.println("Total space " + (getNumberOfEntries() * getNumberOfVariables() * 8) / (1024 * 1024) + " MB.");

      if (bufferedElements > getNumberOfEntries())
      {
         out.println("Reading all elements into buffer");
         bufferedElements = getNumberOfEntries();
      }

      createBuffers();

      decompressData();

      copyMetaData(targetDirectory);

      writeCompressionProperties(targetDirectory);

      close();
      progressMonitor.close();
   }

   private void incrementProgressMonitor()
   {
      progressMonitor.setProgress(progress.incrementAndGet());
   }

   private void createBuffers()
   {
      for (int i = 0; i < NUMBER_OF_THREADS; i++)
      {
         try
         {
            availableBuffers.put(new DataBuffer(getNumberOfVariables(), bufferedElements * 8));
         }
         catch (InterruptedException e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   private void writeCompressionProperties(File targetDirectory) throws IOException, FileNotFoundException
   {
      CompressionProperties properties = new CompressionProperties();
      properties.setNumberOfBufferedElements(bufferedElements);
      properties.setCompressedDataFiles(compressedDataFiles);
      setChecksums(properties);

      FileOutputStream propertiesFile = new FileOutputStream(new File(targetDirectory, "robotData.compressed"));
      properties.store(propertiesFile, "Description of compression properties");
      propertiesFile.close();
   }

   private void setChecksums(CompressionProperties properties) throws IOException
   {
      out.println("Saving checksums to file");
      File logdata = new File(logDirectory, logProperties.getVariables().getDataAsString());
      properties.setDataChecksum(Files.hash(logdata, Hashing.sha1()).toString());

      File index = new File(logDirectory, logProperties.getVariables().getDataAsString());
      properties.setTimestampChecksum(Files.hash(index, Hashing.sha1()).toString());
      out.println("Saved checksums");
   }

   private void decompressData() throws IOException
   {

      int fileIndex = 0;
      int currentFileEntry = 0;

      DataBuffer buffer = getFreeDataBuffer(fileIndex);

      for (int i = 0; i < getNumberOfEntries(); i++)
      {
         ByteBuffer dataLine = readData(i);

         for (int e = 0; e < getNumberOfVariables(); e++)
         {
            buffer.data[e][currentFileEntry * 8] = dataLine.get();
            buffer.data[e][currentFileEntry * 8 + 1] = dataLine.get();
            buffer.data[e][currentFileEntry * 8 + 2] = dataLine.get();
            buffer.data[e][currentFileEntry * 8 + 3] = dataLine.get();
            buffer.data[e][currentFileEntry * 8 + 4] = dataLine.get();
            buffer.data[e][currentFileEntry * 8 + 5] = dataLine.get();
            buffer.data[e][currentFileEntry * 8 + 6] = dataLine.get();
            buffer.data[e][currentFileEntry * 8 + 7] = dataLine.get();
         }

         currentFileEntry++;
         if (currentFileEntry >= bufferedElements)
         {
            submitBuffer(buffer, currentFileEntry);
            fileIndex++;
            buffer = getFreeDataBuffer(fileIndex);
            currentFileEntry = 0;
         }
      }
      if(currentFileEntry != 0)
      {
         submitBuffer(buffer, currentFileEntry);
      }

      out.println("All data read");
      threadPool.shutdown();

   }

   private void submitBuffer(DataBuffer buffer, int numberOfEntries)
   {
      try
      {
         buffer.length = numberOfEntries;
         writableBuffers.put(buffer);
         threadPool.execute(new DataWriter());
         compressedDataFiles++;
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }

   private DataBuffer getFreeDataBuffer(int currentFile)
   {
      try
      {
         DataBuffer dataBuffers = availableBuffers.take();

         dataBuffers.fileID = currentFile;
         return dataBuffers;
      }
      catch (InterruptedException e1)
      {
         throw new RuntimeException(e1);
      }
   }

   private class DataBuffer
   {
      private final byte[][] data;
      private int fileID;
      private int length;

      public DataBuffer(int variables, int elements)
      {
         data = new byte[variables][elements];
      }
   }

   private class DataWriter implements Runnable
   {

      @Override
      public void run()
      {
         try
         {
            DataBuffer buffer = writableBuffers.take();
            int currentFile = buffer.fileID;
            byte[][] dataBuffers = buffer.data;

            FileOutputStream logOutputStream = new FileOutputStream(new File(targetDirectory, "robotData." + currentFile + ".xz"));
            XZOutputStream xzOutputStream = new XZOutputStream(logOutputStream, new LZMA2Options(COMPRESSION_LEVEL));
            int writtenLength = 0;
            for (int i = 0; i < getNumberOfVariables(); i++)
            {
               writtenLength += buffer.length * 8;
               xzOutputStream.write(dataBuffers[i], 0, buffer.length * 8);
            }

            out.println("Wrote " + writtenLength / (1024 * 1024) + " MB to robotData." + currentFile + ".xz");
            xzOutputStream.close();
            logOutputStream.close();
            availableBuffers.put(buffer);
            incrementProgressMonitor();
         }
         catch (IOException | InterruptedException e)
         {
            e.printStackTrace();
         }

      }

   }

   public static void main(String[] args) throws IOException
   {
      //      File logDirectory = FileSelectionDialog.loadDirectoryWithFileNamed(YoVariableLoggerListener.propertyFile);
      File logDirectory = new File("/home/jesper/robotLogs/20160520_130501_Atlas_WalkOnStraightLinesFallStanding_Step/");
      File targetDirectory = new File("/home/jesper/scratch/compressed/");
      if (logDirectory != null)
      {
         LogPropertiesReader logProperties = new LogPropertiesReader(new File(logDirectory, YoVariableLoggerListener.propertyFile));

         new LogFileCompressor(logDirectory, targetDirectory, logProperties, new CustomProgressMonitor());
      }
   }
}

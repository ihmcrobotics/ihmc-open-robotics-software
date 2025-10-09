package us.ihmc.perception.gpuMapping;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.ShortBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class HeightMapLogger
{
   private final HeightMapParameters heightMapParameters;
   private FileOutputStream heightMapOutputStream;

   public HeightMapLogger(HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;
   }

   public static float[] packArrayForFile(Mat heightMap, Point3D gridCenter, float widthInMeters, float cellSizeInMeters)
   {
      // Snap to cell resolution
      widthInMeters = (float) (Math.floor(widthInMeters / cellSizeInMeters) * cellSizeInMeters);
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex + 1;
      int totalCells = cellsPerAxis * cellsPerAxis;

      // Ensure the Mat is a 16-bit unsigned single channel
      if (heightMap.type() != opencv_core.CV_16UC1)
         throw new IllegalArgumentException("Expected CV_16UC1 Mat");

      // Read the short values from the Mat
      ShortBuffer shortBuffer = heightMap.createBuffer();
      short[] shortHeights = new short[totalCells];
      shortBuffer.get(shortHeights);

      // Prepare an output array with a header
      final int headerFloats = 4;
      float[] packedArray = new float[headerFloats + totalCells];

      // Write header
      packedArray[0] = widthInMeters;
      packedArray[1] = cellSizeInMeters;
      packedArray[2] = (float) gridCenter.getX();
      packedArray[3] = (float) gridCenter.getY();

      // Convert shorts to floats and copy into a packed array
      for (int i = 0; i < totalCells; ++i)
      {
         packedArray[headerFloats + i] = shortHeights[i];
      }

      return packedArray;
   }

   public static void logHeightMapToFile(FileOutputStream fos, float[] packedArray, double timestamp) throws IOException
   {
      int frameSize = 8 + packedArray.length * Float.BYTES;

      ByteBuffer buffer = ByteBuffer.allocate(4 + frameSize);
      buffer.order(ByteOrder.LITTLE_ENDIAN);

      // Write frame size (int)
      buffer.putInt(frameSize);

      // Write timestamp (double)
      buffer.putDouble(timestamp);

      // Write packed float data
      for (float f : packedArray)
      {
         buffer.putFloat(f);
      }

      // Write to file
      fos.write(buffer.array());
   }

   public void logHeightMap(Mat globalHeightMap, Point3D heightMapCenterPoint)
   {
      if (heightMapParameters.getLogHeightMap())
      {
         float[] floatsToLog = packArrayForFile(globalHeightMap,
                                                heightMapCenterPoint,
                                                (float) heightMapParameters.getWidthInMeters(),
                                                (float) heightMapParameters.getCellSize());
         try
         {
            String timestamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS"));

            Path heightMapDirectory = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY;

            Path binaryLogPath = heightMapDirectory.resolve(timestamp + "_HeightMapLog.bin");
            if (heightMapOutputStream == null)
            {
               heightMapOutputStream = new FileOutputStream(binaryLogPath.toFile(), true);
               LogTools.info("Writing height map log to " + binaryLogPath);
            }
            if (!Files.exists(heightMapDirectory))
            {
               Files.createDirectory(heightMapDirectory);
            }
         }
         catch (FileNotFoundException e)
         {
            throw new RuntimeException(e);
         }
         catch (IOException ignored)
         {
         }

         try
         {
            logHeightMapToFile(heightMapOutputStream, floatsToLog, System.currentTimeMillis());
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }

      if (!heightMapParameters.getLogHeightMap())
      {
         if (heightMapOutputStream != null)
         {
            try
            {
               heightMapOutputStream.close();
               heightMapOutputStream = null;
            }
            catch (IOException e)
            {
               throw new RuntimeException(e);
            }
         }
      }
   }
}

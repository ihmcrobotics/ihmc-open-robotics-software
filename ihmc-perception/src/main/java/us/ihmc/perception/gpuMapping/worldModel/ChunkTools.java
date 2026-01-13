package us.ihmc.perception.gpuMapping.worldModel;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.gpuMapping.HeightMapTools;

import java.nio.FloatBuffer;

public class ChunkTools
{
   public static void convertToChunk(Mat chunkMapPointer, Chunk chunkToPack, Point3D gridCenter, float widthInMeters, float cellSizeInMeters)
   {
      widthInMeters = (float) (Math.floor(widthInMeters / cellSizeInMeters) * cellSizeInMeters);
      int centerIndex = HeightMapTools.computeCenterIndex(widthInMeters, cellSizeInMeters);
      int cellsPerAxis = 2 * centerIndex;
      int totalCells = cellsPerAxis * cellsPerAxis;

      chunkToPack.setOriginX(gridCenter.getX());
      chunkToPack.setOriginY(gridCenter.getY());

      FloatPointer floatPointer = new FloatPointer(chunkMapPointer.data());
      float[] values = new float[totalCells];
      floatPointer.get(values);

      chunkToPack.setChunkHeights(values);
   }

   public static void convertToMat(Mat map, Chunk chunk)
   {
      FloatBuffer floatBuffer = map.createBuffer();
      float[] heights = chunk.getChunkHeights();
      floatBuffer.put(heights);
   }
}

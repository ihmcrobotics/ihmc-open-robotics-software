package us.ihmc.rdx.ui.graphics.live;

import org.bytedeco.javacpp.FloatPointer;

public class PointCloud
{
   private final int numberOfPoints;
   private final int numberOfElementsPerPoint;
   private final float[] data;

   public PointCloud(int numberOfPoints, int pointLength)
   {
      this.numberOfPoints = numberOfPoints;
      this.numberOfElementsPerPoint = pointLength;
      data = new float[numberOfPoints * pointLength];
   }

   public void setData(FloatPointer fp, int size)
   {
      fp.get(data, 0, size);
   }

   public float[] getData()
   {
      return this.data;
   }

   public int getNumberOfPoints()
   {
      return numberOfPoints;
   }

   public int getNumberOfElementsPerPoint()
   {
      return numberOfElementsPerPoint;
   }
}

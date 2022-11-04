package us.ihmc.perception.elements;

import org.bytedeco.javacpp.FloatPointer;

public class PointCloud
{
   private final int numberOfPoints;
   private final int numberOfElementsPerPoint;
   private final float[] data;

   public PointCloud(int numberOfPoints, int numberOfElementsPerPoint)
   {
      this.numberOfPoints = numberOfPoints;
      this.numberOfElementsPerPoint = numberOfElementsPerPoint;
      data = new float[numberOfPoints * numberOfElementsPerPoint];
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

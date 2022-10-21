package us.ihmc.rdx.ui.graphics.live;

import org.bytedeco.javacpp.FloatPointer;

public class PointCloud
{
   private final int numberOfPoints;
   private final int pointLength;
   private final float[] data;

   public PointCloud(int numberOfPoints, int pointLength)
   {
      this.numberOfPoints = numberOfPoints;
      this.pointLength = pointLength;
      data = new float[numberOfPoints * pointLength];
   }

   public void setData(FloatPointer fp, int size)
   {
      for (int i = 0; i < size; ++i)
      {
         data[i] = fp.get(i);
      }
   }

   public float[] getData()
   {
      return this.data;
   }

   public int getNumberOfPoints()
   {
      return numberOfPoints;
   }

   public int getPointLength()
   {
      return pointLength;
   }
}

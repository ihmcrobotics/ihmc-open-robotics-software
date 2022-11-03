package us.ihmc.rdx.ui.graphics.live;

import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.perception.OpenCLIntBuffer;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;

public class PointCloud
{
   private final int numberOfPoints;
   private final int numberOfElementsPerPoint;
   private final ByteBuffer data;  // x,y,z,color

   public PointCloud(int numberOfPoints, int numberOfElementsPerPoint)
   {
      this.numberOfPoints = numberOfPoints;
      this.numberOfElementsPerPoint = numberOfElementsPerPoint;
      data = ByteBuffer.allocateDirect(numberOfPoints * numberOfElementsPerPoint * Integer.BYTES);
   }

   public void setData(ByteBuffer fp, int size)
   {
//      for (int i = 0; i < size; ++i)
//      {
         data.put(fp);
//         data[i] = fp.get(i);
//      }
   }

   public void rewindBufferData()
   {
      data.rewind();
   }

   public ByteBuffer getData()
   {
      return this.data;
   }

   public int getNumberOfPoints()
   {
      return numberOfPoints;
   }

   public void limit(int numberOfBytes)
   {
      getData().limit(numberOfBytes);
   }

   public int getNumberOfElementsPerPoint()
   {
      return numberOfElementsPerPoint;
   }
}

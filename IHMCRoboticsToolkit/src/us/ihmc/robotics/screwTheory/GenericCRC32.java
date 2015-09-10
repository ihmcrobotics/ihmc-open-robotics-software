package us.ihmc.robotics.screwTheory;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.zip.CRC32;

import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple4d;

import org.ejml.data.DenseMatrix64F;



public class GenericCRC32 extends CRC32
{
   private final byte[] byteArray = new byte[8];
   private final ByteBuffer byteBuffer = ByteBuffer.wrap(byteArray);
   
   private final DoubleBuffer doubleBuffer = byteBuffer.asDoubleBuffer();
   private final LongBuffer longBuffer = byteBuffer.asLongBuffer();
   private final IntBuffer intBuffer = byteBuffer.asIntBuffer();  
   
   
   public void update(double value)
   {
      doubleBuffer.put(0, value);
      update(byteArray);
   }
   
   public void updateInteger(int value)
   {
      intBuffer.put(0, value);
      update(byteArray, 0, 4);
   }
   
   public void update(long value)
   {
      longBuffer.put(value);
      update(byteArray);
   }
   
   public void update(Tuple2d value)
   {
      update(value.getX());
      update(value.getY());
   }
   
   public void update(Tuple3d value)
   {
      update(value.getX());
      update(value.getY());
      update(value.getZ());
   }
   
   public void update(Tuple4d value)
   {
      update(value.getW());
      update(value.getX());
      update(value.getY());
      update(value.getZ());
   }
   
   public void update(boolean value)
   {
      update(value?0x1:0x0);
   }

   public void update(String value)
   {
      update(value.hashCode());
   }

   public void update(DenseMatrix64F value)
   {
      for(int i = 0; i < value.getNumElements(); i++)
      {
         update(value.get(i));
      }
   }
}

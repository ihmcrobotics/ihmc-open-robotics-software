package us.ihmc.robotics.screwTheory;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.zip.CRC32;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;



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
   
   public void update(Tuple2DReadOnly value)
   {
      update(value.getX());
      update(value.getY());
   }
   
   public void update(Tuple3DReadOnly value)
   {
      update(value.getX());
      update(value.getY());
      update(value.getZ());
   }
   
   public void update(Tuple4DReadOnly value)
   {
      update(value.getS());
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

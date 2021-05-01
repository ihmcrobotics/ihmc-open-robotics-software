package us.ihmc.communication.net;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.euclid.tuple3D.Point3D;

public class Point3DSerializer extends Serializer<Point3D>
{

   @Override
   public void write(Kryo kryo, Output output, Point3D object)
   {
      output.writeDouble(object.getX());
      output.writeDouble(object.getY());
      output.writeDouble(object.getZ());
   }

   @Override
   public Point3D read(Kryo kryo, Input input, Class<? extends Point3D> type)
   {
      return new Point3D(input.readDouble(), input.readDouble(), input.readDouble());
   }

   @Override
   public Point3D copy(Kryo kryo, Point3D original)
   {
      return new Point3D(original);
   }
}

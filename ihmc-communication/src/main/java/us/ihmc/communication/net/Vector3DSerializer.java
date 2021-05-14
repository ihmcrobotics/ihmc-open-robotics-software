package us.ihmc.communication.net;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.euclid.tuple3D.Vector3D;

public class Vector3DSerializer extends Serializer<Vector3D>
{

   @Override
   public void write(Kryo kryo, Output output, Vector3D object)
   {
      output.writeDouble(object.getX());
      output.writeDouble(object.getY());
      output.writeDouble(object.getZ());
   }

   @Override
   public Vector3D read(Kryo kryo, Input input, Class<? extends Vector3D> type)
   {
      return new Vector3D(input.readDouble(), input.readDouble(), input.readDouble());
   }

   @Override
   public Vector3D copy(Kryo kryo, Vector3D original)
   {
      return new Vector3D(original);
   }
}

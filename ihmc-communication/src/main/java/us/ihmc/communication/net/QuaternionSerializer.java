package us.ihmc.communication.net;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.euclid.tuple4D.Quaternion;

public class QuaternionSerializer extends Serializer<Quaternion>
{

   @Override
   public void write(Kryo kryo, Output output, Quaternion object)
   {
      output.writeDouble(object.getX());
      output.writeDouble(object.getY());
      output.writeDouble(object.getZ());
      output.writeDouble(object.getS());
   }

   @Override
   public Quaternion read(Kryo kryo, Input input, Class<? extends Quaternion> type)
   {
      return new Quaternion(input.readDouble(), input.readDouble(), input.readDouble(), input.readDouble());
   }

   @Override
   public Quaternion copy(Kryo kryo, Quaternion original)
   {
      return new Quaternion(original);
   }
}

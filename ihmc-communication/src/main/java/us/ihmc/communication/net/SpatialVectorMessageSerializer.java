package us.ihmc.communication.net;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.euclid.tuple3D.Vector3D;

public class SpatialVectorMessageSerializer extends Serializer<SpatialVectorMessage>
{
   @Override
   public void write(Kryo kryo, Output output, SpatialVectorMessage object)
   {
      Vector3D angularPart = object.getAngularPart();
      Vector3D linearPart = object.getLinearPart();

      output.writeInt(object.getSource());
      output.writeInt(object.getDestination());
      output.writeLong(object.getUniqueId());
      output.writeLong(object.getSequenceId());

      output.writeDouble(angularPart.getX());
      output.writeDouble(angularPart.getY());
      output.writeDouble(angularPart.getZ());

      output.writeDouble(linearPart.getX());
      output.writeDouble(linearPart.getY());
      output.writeDouble(linearPart.getZ());
   }

   @Override
   public SpatialVectorMessage read(Kryo kryo, Input input, Class<? extends SpatialVectorMessage> type)
   {
      SpatialVectorMessage message = new SpatialVectorMessage();

      message.setSource(input.readInt());
      message.setDestination(input.readInt());
      message.setUniqueId(input.readLong());
      message.setSequenceId(input.readLong());

      Vector3D angularPart = message.getAngularPart();
      angularPart.setX(input.readDouble());
      angularPart.setY(input.readDouble());
      angularPart.setZ(input.readDouble());
      Vector3D linearPart = message.getLinearPart();
      linearPart.setX(input.readDouble());
      linearPart.setY(input.readDouble());
      linearPart.setZ(input.readDouble());

      return message;
   }

   @Override
   public SpatialVectorMessage copy(Kryo kryo, SpatialVectorMessage original)
   {
      return new SpatialVectorMessage(original);
   }
}

package us.ihmc.communication.net;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import controller_msgs.msg.dds.IMUPacket;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class IMUPacketSerializer extends Serializer<IMUPacket>
{
   @Override
   public void write(Kryo kryo, Output output, IMUPacket object)
   {
      output.writeInt(object.getSource());
      output.writeInt(object.getDestination());
      output.writeLong(object.getUniqueId());
      output.writeLong(object.getSequenceId());

      Quaternion orientation = object.getOrientation();
      output.writeDouble(orientation.getX());
      output.writeDouble(orientation.getY());
      output.writeDouble(orientation.getZ());
      output.writeDouble(orientation.getS());

      Vector3D angularVelocity = object.getAngularVelocity();
      output.writeDouble(angularVelocity.getX());
      output.writeDouble(angularVelocity.getY());
      output.writeDouble(angularVelocity.getZ());

      Vector3D linearAcceleration = object.getLinearAcceleration();
      output.writeDouble(linearAcceleration.getX());
      output.writeDouble(linearAcceleration.getY());
      output.writeDouble(linearAcceleration.getZ());

      output.writeDouble(object.getTime());
   }

   @Override
   public IMUPacket read(Kryo kryo, Input input, Class<? extends IMUPacket> type)
   {
      IMUPacket message = new IMUPacket();

      message.setSource(input.readInt());
      message.setDestination(input.readInt());
      message.setUniqueId(input.readLong());
      message.setSequenceId(input.readLong());

      Quaternion orientation = message.getOrientation();
      double qx = input.readDouble();
      double qy = input.readDouble();
      double qz = input.readDouble();
      double qs = input.readDouble();
      orientation.setUnsafe(qx, qy, qz, qs);

      Vector3D angularVelocity = message.getAngularVelocity();
      angularVelocity.setX(input.readDouble());
      angularVelocity.setY(input.readDouble());
      angularVelocity.setZ(input.readDouble());

      Vector3D linearAcceleration = message.getLinearAcceleration();
      linearAcceleration.setX(input.readDouble());
      linearAcceleration.setY(input.readDouble());
      linearAcceleration.setZ(input.readDouble());

      message.setTime(input.readDouble());

      return message;
   }

   @Override
   public IMUPacket copy(Kryo kryo, IMUPacket original)
   {
      return new IMUPacket(original);
   }
}

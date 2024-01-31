package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ComPositionPacket" defined in "ComPositionPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ComPositionPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ComPositionPacket_.idl instead.
*
*/
public class ComPositionPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ComPositionPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ComPositionPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5b99bef48b8570d163863aa3bd31234341ea3d0dd74881f074f9451dacafc40e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ComPositionPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ComPositionPacket data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ComPositionPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ComPositionPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ComPositionPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);   }

   public static void read(controller_msgs.msg.dds.ComPositionPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ComPositionPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ComPositionPacket data)
   {
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());
   }

   public static void staticCopy(controller_msgs.msg.dds.ComPositionPacket src, controller_msgs.msg.dds.ComPositionPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ComPositionPacket createData()
   {
      return new controller_msgs.msg.dds.ComPositionPacket();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(controller_msgs.msg.dds.ComPositionPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ComPositionPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ComPositionPacket src, controller_msgs.msg.dds.ComPositionPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ComPositionPacketPubSubType newInstance()
   {
      return new ComPositionPacketPubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PlanarVelocityPacket" defined in "PlanarVelocityPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PlanarVelocityPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PlanarVelocityPacket_.idl instead.
*
*/
public class PlanarVelocityPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PlanarVelocityPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PlanarVelocityPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "91871c8e9669b85fe7f2752c9c868face9b972cbbd589da86ca8669dab93bb61";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PlanarVelocityPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PlanarVelocityPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarVelocityPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarVelocityPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getVelocity(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PlanarVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getVelocity(), cdr);   }

   public static void read(controller_msgs.msg.dds.PlanarVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getVelocity(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PlanarVelocityPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getVelocity());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PlanarVelocityPacket data)
   {
      ser.read_type_a("velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getVelocity());
   }

   public static void staticCopy(controller_msgs.msg.dds.PlanarVelocityPacket src, controller_msgs.msg.dds.PlanarVelocityPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PlanarVelocityPacket createData()
   {
      return new controller_msgs.msg.dds.PlanarVelocityPacket();
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
   
   public void serialize(controller_msgs.msg.dds.PlanarVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PlanarVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PlanarVelocityPacket src, controller_msgs.msg.dds.PlanarVelocityPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PlanarVelocityPacketPubSubType newInstance()
   {
      return new PlanarVelocityPacketPubSubType();
   }
}

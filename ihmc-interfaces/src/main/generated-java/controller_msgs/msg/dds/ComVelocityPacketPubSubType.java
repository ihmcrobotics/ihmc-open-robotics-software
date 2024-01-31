package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ComVelocityPacket" defined in "ComVelocityPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ComVelocityPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ComVelocityPacket_.idl instead.
*
*/
public class ComVelocityPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ComVelocityPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ComVelocityPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bf4b8f53d9b069abbbb9154db42cc5bdd97d78a39562ca47f8eaf8bfe6a0139c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ComVelocityPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ComVelocityPacket data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ComVelocityPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ComVelocityPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getVelocity(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ComVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getVelocity(), cdr);   }

   public static void read(controller_msgs.msg.dds.ComVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getVelocity(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ComVelocityPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getVelocity());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ComVelocityPacket data)
   {
      ser.read_type_a("velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getVelocity());
   }

   public static void staticCopy(controller_msgs.msg.dds.ComVelocityPacket src, controller_msgs.msg.dds.ComVelocityPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ComVelocityPacket createData()
   {
      return new controller_msgs.msg.dds.ComVelocityPacket();
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
   
   public void serialize(controller_msgs.msg.dds.ComVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ComVelocityPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ComVelocityPacket src, controller_msgs.msg.dds.ComVelocityPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ComVelocityPacketPubSubType newInstance()
   {
      return new ComVelocityPacketPubSubType();
   }
}

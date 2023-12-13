package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Box3DMessage" defined in "Box3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Box3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Box3DMessage_.idl instead.
*
*/
public class Box3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.Box3DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::Box3DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c18555c54ca0e44a55ee0b8c3591121d2d13ea522b319b183975cd84d1c71e3d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.Box3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.Box3DMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Box3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Box3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getSize(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.Box3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getSize(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.Box3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getSize(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.Box3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("size", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getSize());

      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.Box3DMessage data)
   {
      ser.read_type_a("size", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getSize());

      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.Box3DMessage src, ihmc_common_msgs.msg.dds.Box3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.Box3DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.Box3DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.Box3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.Box3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.Box3DMessage src, ihmc_common_msgs.msg.dds.Box3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Box3DMessagePubSubType newInstance()
   {
      return new Box3DMessagePubSubType();
   }
}

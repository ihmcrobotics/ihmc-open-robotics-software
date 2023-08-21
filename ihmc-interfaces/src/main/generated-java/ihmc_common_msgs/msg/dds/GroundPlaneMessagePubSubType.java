package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GroundPlaneMessage" defined in "GroundPlaneMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GroundPlaneMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GroundPlaneMessage_.idl instead.
*
*/
public class GroundPlaneMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.GroundPlaneMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::GroundPlaneMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ed19af7d7c3bc34bb786f6a4999564a3d394808a87981e48abf81e4f809affef";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.GroundPlaneMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.GroundPlaneMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.GroundPlaneMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.GroundPlaneMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRegionOrigin(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRegionNormal(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.GroundPlaneMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getRegionOrigin(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRegionNormal(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.GroundPlaneMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRegionOrigin(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRegionNormal(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.GroundPlaneMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());

      ser.write_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.GroundPlaneMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("region_origin", new geometry_msgs.msg.dds.PointPubSubType(), data.getRegionOrigin());

      ser.read_type_a("region_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRegionNormal());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.GroundPlaneMessage src, ihmc_common_msgs.msg.dds.GroundPlaneMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.GroundPlaneMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.GroundPlaneMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.GroundPlaneMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.GroundPlaneMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.GroundPlaneMessage src, ihmc_common_msgs.msg.dds.GroundPlaneMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GroundPlaneMessagePubSubType newInstance()
   {
      return new GroundPlaneMessagePubSubType();
   }
}

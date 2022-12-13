package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TimestampedPlanarRegionsListMessage" defined in "TimestampedPlanarRegionsListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TimestampedPlanarRegionsListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TimestampedPlanarRegionsListMessage_.idl instead.
*
*/
public class TimestampedPlanarRegionsListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::TimestampedPlanarRegionsListMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getPlanarRegions(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getLastUpdatedSecondsSinceEpoch());

      cdr.write_type_11(data.getLastUpdatedAdditionalNanos());

      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.write(data.getPlanarRegions(), cdr);
   }

   public static void read(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setLastUpdatedSecondsSinceEpoch(cdr.read_type_11());
      	
      data.setLastUpdatedAdditionalNanos(cdr.read_type_11());
      	
      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.read(data.getPlanarRegions(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("last_updated_seconds_since_epoch", data.getLastUpdatedSecondsSinceEpoch());
      ser.write_type_11("last_updated_additional_nanos", data.getLastUpdatedAdditionalNanos());
      ser.write_type_a("planar_regions", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegions());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data)
   {
      data.setLastUpdatedSecondsSinceEpoch(ser.read_type_11("last_updated_seconds_since_epoch"));
      data.setLastUpdatedAdditionalNanos(ser.read_type_11("last_updated_additional_nanos"));
      ser.read_type_a("planar_regions", new perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType(), data.getPlanarRegions());

   }

   public static void staticCopy(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage src, perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage createData()
   {
      return new perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage();
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
   
   public void serialize(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage src, perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TimestampedPlanarRegionsListMessagePubSubType newInstance()
   {
      return new TimestampedPlanarRegionsListMessagePubSubType();
   }
}

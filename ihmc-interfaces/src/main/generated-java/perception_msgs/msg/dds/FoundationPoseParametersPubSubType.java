package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FoundationPoseParameters" defined in "FoundationPoseParameters_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FoundationPoseParameters_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FoundationPoseParameters_.idl instead.
*
*/
public class FoundationPoseParametersPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.FoundationPoseParameters>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::FoundationPoseParameters_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c1434035f5010fa963672557adc3704549a436ed42e31980103e3284091dd023";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.FoundationPoseParameters data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.FoundationPoseParameters data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseParameters data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.FoundationPoseParameters data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestTimestampModifiable(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.FoundationPoseParameters data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestTimestampModifiable(), cdr);
      cdr.write_type_7(data.getEnabled());

      cdr.write_type_7(data.getAutoResetEnabled());

      cdr.write_type_6(data.getResetDistance());

   }

   public static void read(perception_msgs.msg.dds.FoundationPoseParameters data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestTimestampModifiable(), cdr);	
      data.setEnabled(cdr.read_type_7());
      	
      data.setAutoResetEnabled(cdr.read_type_7());
      	
      data.setResetDistance(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.FoundationPoseParameters data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      ser.write_type_7("enabled", data.getEnabled());
      ser.write_type_7("auto_reset_enabled", data.getAutoResetEnabled());
      ser.write_type_6("reset_distance", data.getResetDistance());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.FoundationPoseParameters data)
   {
      ser.read_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      data.setEnabled(ser.read_type_7("enabled"));
      data.setAutoResetEnabled(ser.read_type_7("auto_reset_enabled"));
      data.setResetDistance(ser.read_type_6("reset_distance"));
   }

   public static void staticCopy(perception_msgs.msg.dds.FoundationPoseParameters src, perception_msgs.msg.dds.FoundationPoseParameters dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.FoundationPoseParameters createData()
   {
      return new perception_msgs.msg.dds.FoundationPoseParameters();
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
   
   public void serialize(perception_msgs.msg.dds.FoundationPoseParameters data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.FoundationPoseParameters data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.FoundationPoseParameters src, perception_msgs.msg.dds.FoundationPoseParameters dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FoundationPoseParametersPubSubType newInstance()
   {
      return new FoundationPoseParametersPubSubType();
   }
}

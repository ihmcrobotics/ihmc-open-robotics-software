package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactTimedContactSequenceMessage" defined in "MultiContactTimedContactSequenceMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactTimedContactSequenceMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactTimedContactSequenceMessage_.idl instead.
*
*/
public class MultiContactTimedContactSequenceMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::MultiContactTimedContactSequenceMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "356e04a1322d0b50a85e2802c74a9a67ec241dedf2acdf921717cb366c1abd09";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLeftArmContactIntervals().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getCdrSerializedSize(data.getLeftArmContactIntervals().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRightArmContactIntervals().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getCdrSerializedSize(data.getRightArmContactIntervals().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLeftLegContactIntervals().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getCdrSerializedSize(data.getLeftLegContactIntervals().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRightLegContactIntervals().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.TimeIntervalMessagePubSubType.getCdrSerializedSize(data.getRightLegContactIntervals().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getLeftArmContactIntervals().size() <= 25)
      cdr.write_type_e(data.getLeftArmContactIntervals());else
          throw new RuntimeException("left_arm_contact_intervals field exceeds the maximum length");

      if(data.getRightArmContactIntervals().size() <= 25)
      cdr.write_type_e(data.getRightArmContactIntervals());else
          throw new RuntimeException("right_arm_contact_intervals field exceeds the maximum length");

      if(data.getLeftLegContactIntervals().size() <= 25)
      cdr.write_type_e(data.getLeftLegContactIntervals());else
          throw new RuntimeException("left_leg_contact_intervals field exceeds the maximum length");

      if(data.getRightLegContactIntervals().size() <= 25)
      cdr.write_type_e(data.getRightLegContactIntervals());else
          throw new RuntimeException("right_leg_contact_intervals field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getLeftArmContactIntervals());	
      cdr.read_type_e(data.getRightArmContactIntervals());	
      cdr.read_type_e(data.getLeftLegContactIntervals());	
      cdr.read_type_e(data.getRightLegContactIntervals());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("left_arm_contact_intervals", data.getLeftArmContactIntervals());
      ser.write_type_e("right_arm_contact_intervals", data.getRightArmContactIntervals());
      ser.write_type_e("left_leg_contact_intervals", data.getLeftLegContactIntervals());
      ser.write_type_e("right_leg_contact_intervals", data.getRightLegContactIntervals());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("left_arm_contact_intervals", data.getLeftArmContactIntervals());
      ser.read_type_e("right_arm_contact_intervals", data.getRightArmContactIntervals());
      ser.read_type_e("left_leg_contact_intervals", data.getLeftLegContactIntervals());
      ser.read_type_e("right_leg_contact_intervals", data.getRightLegContactIntervals());
   }

   public static void staticCopy(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage src, controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage createData()
   {
      return new controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage();
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
   
   public void serialize(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage src, controller_msgs.msg.dds.MultiContactTimedContactSequenceMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactTimedContactSequenceMessagePubSubType newInstance()
   {
      return new MultiContactTimedContactSequenceMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "FootstepList" defined in "FootstepList_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from FootstepList_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit FootstepList_.idl instead.
 */
public class FootstepListPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepList>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepList_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public FootstepListPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < 100; ++i0)
      {
         current_alignment += controller_msgs.msg.dds.FootstepPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepList data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepList data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int i0 = 0; i0 < data.getFootsteps().size(); ++i0)
      {
         current_alignment += controller_msgs.msg.dds.FootstepPubSubType.getCdrSerializedSize(data.getFootsteps().get(i0), current_alignment);
      }

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepList data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_12(data.getUniqueId());

      if (data.getFootsteps().size() <= 100)
         cdr.write_type_e(data.getFootsteps());
      else
         throw new RuntimeException("footsteps field exceeds the maximum length");

      cdr.write_type_9(data.getExecutingTiming());

      cdr.write_type_6(data.getDefaultSwingDuration());

      cdr.write_type_6(data.getFinalTransferDuration());
   }

   public static void read(controller_msgs.msg.dds.FootstepList data, us.ihmc.idl.CDR cdr)
   {

      data.setUniqueId(cdr.read_type_12());

      cdr.read_type_e(data.getFootsteps());

      data.setExecutingTiming(cdr.read_type_9());

      data.setDefaultSwingDuration(cdr.read_type_6());

      data.setFinalTransferDuration(cdr.read_type_6());
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepList src, controller_msgs.msg.dds.FootstepList dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepList data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepList data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepList data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("unique_id", data.getUniqueId());

      ser.write_type_e("footsteps", data.getFootsteps());

      ser.write_type_9("executing_timing", data.getExecutingTiming());

      ser.write_type_6("default_swing_duration", data.getDefaultSwingDuration());

      ser.write_type_6("final_transfer_duration", data.getFinalTransferDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepList data)
   {
      data.setUniqueId(ser.read_type_12("unique_id"));

      ser.read_type_e("footsteps", data.getFootsteps());

      data.setExecutingTiming(ser.read_type_9("executing_timing"));

      data.setDefaultSwingDuration(ser.read_type_6("default_swing_duration"));

      data.setFinalTransferDuration(ser.read_type_6("final_transfer_duration"));
   }

   @Override
   public controller_msgs.msg.dds.FootstepList createData()
   {
      return new controller_msgs.msg.dds.FootstepList();
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

   public void serialize(controller_msgs.msg.dds.FootstepList data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepList data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.FootstepList src, controller_msgs.msg.dds.FootstepList dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepListPubSubType newInstance()
   {
      return new FootstepListPubSubType();
   }
}
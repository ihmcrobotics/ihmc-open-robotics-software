package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "PoseWithConfidenceStamped" defined in "PoseWithConfidenceStamped_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from PoseWithConfidenceStamped_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit PoseWithConfidenceStamped_.idl instead.
 */
public class PoseWithConfidenceStampedPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PoseWithConfidenceStamped>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PoseWithConfidenceStamped_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public PoseWithConfidenceStampedPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PoseStampedPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PoseWithConfidenceStamped data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PoseWithConfidenceStamped data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PoseStampedPubSubType.getCdrSerializedSize(data.getPose(), current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PoseWithConfidenceStamped data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.PoseStampedPubSubType.write(data.getPose(), cdr);

      cdr.write_type_5(data.getConfidence());
   }

   public static void read(controller_msgs.msg.dds.PoseWithConfidenceStamped data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.PoseStampedPubSubType.read(data.getPose(), cdr);

      data.setConfidence(cdr.read_type_5());
   }

   public static void staticCopy(controller_msgs.msg.dds.PoseWithConfidenceStamped src, controller_msgs.msg.dds.PoseWithConfidenceStamped dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.PoseWithConfidenceStamped data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PoseWithConfidenceStamped data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PoseWithConfidenceStamped data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("pose", new geometry_msgs.msg.dds.PoseStampedPubSubType(), data.getPose());

      ser.write_type_5("confidence", data.getConfidence());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PoseWithConfidenceStamped data)
   {
      ser.read_type_a("pose", new geometry_msgs.msg.dds.PoseStampedPubSubType(), data.getPose());

      data.setConfidence(ser.read_type_5("confidence"));
   }

   @Override
   public controller_msgs.msg.dds.PoseWithConfidenceStamped createData()
   {
      return new controller_msgs.msg.dds.PoseWithConfidenceStamped();
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

   public void serialize(controller_msgs.msg.dds.PoseWithConfidenceStamped data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PoseWithConfidenceStamped data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.PoseWithConfidenceStamped src, controller_msgs.msg.dds.PoseWithConfidenceStamped dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PoseWithConfidenceStampedPubSubType newInstance()
   {
      return new PoseWithConfidenceStampedPubSubType();
   }
}
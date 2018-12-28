package controller_msgs.msg.dds;

/**
 *
 * Topic data type of the struct "QuadrupedStepMessage" defined in "QuadrupedStepMessage_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from QuadrupedStepMessage_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit QuadrupedStepMessage_.idl instead.
 *
 */
public class QuadrupedTeleopDesiredPosePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedTeleopDesiredPose>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedTeleopDesiredPose_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);

   }

   public static void read(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());

      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose src, controller_msgs.msg.dds.QuadrupedTeleopDesiredPose dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedTeleopDesiredPose createData()
   {
      return new controller_msgs.msg.dds.QuadrupedTeleopDesiredPose();
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

   public void serialize(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.QuadrupedTeleopDesiredPose src, controller_msgs.msg.dds.QuadrupedTeleopDesiredPose dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedTeleopDesiredPosePubSubType newInstance()
   {
      return new QuadrupedTeleopDesiredPosePubSubType();
   }
}
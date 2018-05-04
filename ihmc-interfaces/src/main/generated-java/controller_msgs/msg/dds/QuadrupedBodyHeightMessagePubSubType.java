package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedBodyHeightMessage" defined in "QuadrupedBodyHeightMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedBodyHeightMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedBodyHeightMessage_.idl instead.
*
*/
public class QuadrupedBodyHeightMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedBodyHeightMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedBodyHeightMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedBodyHeightMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getCdrSerializedSize(data.getEuclideanTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getIsExpressedInAbsoluteTime());

      cdr.write_type_7(data.getControlBodyHeight());

      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.write(data.getEuclideanTrajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setIsExpressedInAbsoluteTime(cdr.read_type_7());
      	
      data.setControlBodyHeight(cdr.read_type_7());
      	
      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.read(data.getEuclideanTrajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("is_expressed_in_absolute_time", data.getIsExpressedInAbsoluteTime());
      ser.write_type_7("control_body_height", data.getControlBodyHeight());
      ser.write_type_a("euclidean_trajectory", new controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedBodyHeightMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setIsExpressedInAbsoluteTime(ser.read_type_7("is_expressed_in_absolute_time"));
      data.setControlBodyHeight(ser.read_type_7("control_body_height"));
      ser.read_type_a("euclidean_trajectory", new controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedBodyHeightMessage src, controller_msgs.msg.dds.QuadrupedBodyHeightMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedBodyHeightMessage createData()
   {
      return new controller_msgs.msg.dds.QuadrupedBodyHeightMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedBodyHeightMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedBodyHeightMessage src, controller_msgs.msg.dds.QuadrupedBodyHeightMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedBodyHeightMessagePubSubType newInstance()
   {
      return new QuadrupedBodyHeightMessagePubSubType();
   }
}

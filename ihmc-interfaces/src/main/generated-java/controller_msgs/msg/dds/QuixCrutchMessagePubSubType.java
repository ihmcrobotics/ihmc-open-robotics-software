package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuixCrutchMessage" defined in "QuixCrutchMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuixCrutchMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuixCrutchMessage_.idl instead.
*
*/
public class QuixCrutchMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuixCrutchMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuixCrutchMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuixCrutchMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuixCrutchMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.QuixMotionStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += controller_msgs.msg.dds.FlatStepTypeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.QuixStairsStepTypeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.QuixSideStepDirectionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.QuixSlopeStepTypeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuixCrutchMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuixCrutchMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.QuixMotionStateMessagePubSubType.getCdrSerializedSize(data.getRequestedMotionState(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.FlatStepTypeMessagePubSubType.getCdrSerializedSize(data.getFlatStepType(), current_alignment);

      current_alignment += controller_msgs.msg.dds.QuixStairsStepTypeMessagePubSubType.getCdrSerializedSize(data.getStairsStepType(), current_alignment);

      current_alignment += controller_msgs.msg.dds.QuixSideStepDirectionMessagePubSubType.getCdrSerializedSize(data.getSideStepDirection(), current_alignment);

      current_alignment += controller_msgs.msg.dds.QuixSlopeStepTypeMessagePubSubType.getCdrSerializedSize(data.getSlopeStepType(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuixCrutchMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getUserEnable());

      cdr.write_type_7(data.getRewiggle());

      cdr.write_type_7(data.getStartBehavior());

      controller_msgs.msg.dds.QuixMotionStateMessagePubSubType.write(data.getRequestedMotionState(), cdr);
      cdr.write_type_7(data.getExecuteBehavior());

      cdr.write_type_7(data.getContinuousWalking());

      controller_msgs.msg.dds.FlatStepTypeMessagePubSubType.write(data.getFlatStepType(), cdr);
      controller_msgs.msg.dds.QuixStairsStepTypeMessagePubSubType.write(data.getStairsStepType(), cdr);
      controller_msgs.msg.dds.QuixSideStepDirectionMessagePubSubType.write(data.getSideStepDirection(), cdr);
      controller_msgs.msg.dds.QuixSlopeStepTypeMessagePubSubType.write(data.getSlopeStepType(), cdr);
      cdr.write_type_9(data.getForceSwingSide());

   }

   public static void read(controller_msgs.msg.dds.QuixCrutchMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setUserEnable(cdr.read_type_7());
      	
      data.setRewiggle(cdr.read_type_7());
      	
      data.setStartBehavior(cdr.read_type_7());
      	
      controller_msgs.msg.dds.QuixMotionStateMessagePubSubType.read(data.getRequestedMotionState(), cdr);	
      data.setExecuteBehavior(cdr.read_type_7());
      	
      data.setContinuousWalking(cdr.read_type_7());
      	
      controller_msgs.msg.dds.FlatStepTypeMessagePubSubType.read(data.getFlatStepType(), cdr);	
      controller_msgs.msg.dds.QuixStairsStepTypeMessagePubSubType.read(data.getStairsStepType(), cdr);	
      controller_msgs.msg.dds.QuixSideStepDirectionMessagePubSubType.read(data.getSideStepDirection(), cdr);	
      controller_msgs.msg.dds.QuixSlopeStepTypeMessagePubSubType.read(data.getSlopeStepType(), cdr);	
      data.setForceSwingSide(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuixCrutchMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("user_enable", data.getUserEnable());
      ser.write_type_7("rewiggle", data.getRewiggle());
      ser.write_type_7("start_behavior", data.getStartBehavior());
      ser.write_type_a("requested_motion_state", new controller_msgs.msg.dds.QuixMotionStateMessagePubSubType(), data.getRequestedMotionState());

      ser.write_type_7("execute_behavior", data.getExecuteBehavior());
      ser.write_type_7("continuous_walking", data.getContinuousWalking());
      ser.write_type_a("flat_step_type", new controller_msgs.msg.dds.FlatStepTypeMessagePubSubType(), data.getFlatStepType());

      ser.write_type_a("stairs_step_type", new controller_msgs.msg.dds.QuixStairsStepTypeMessagePubSubType(), data.getStairsStepType());

      ser.write_type_a("side_step_direction", new controller_msgs.msg.dds.QuixSideStepDirectionMessagePubSubType(), data.getSideStepDirection());

      ser.write_type_a("slope_step_type", new controller_msgs.msg.dds.QuixSlopeStepTypeMessagePubSubType(), data.getSlopeStepType());

      ser.write_type_9("force_swing_side", data.getForceSwingSide());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuixCrutchMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setUserEnable(ser.read_type_7("user_enable"));
      data.setRewiggle(ser.read_type_7("rewiggle"));
      data.setStartBehavior(ser.read_type_7("start_behavior"));
      ser.read_type_a("requested_motion_state", new controller_msgs.msg.dds.QuixMotionStateMessagePubSubType(), data.getRequestedMotionState());

      data.setExecuteBehavior(ser.read_type_7("execute_behavior"));
      data.setContinuousWalking(ser.read_type_7("continuous_walking"));
      ser.read_type_a("flat_step_type", new controller_msgs.msg.dds.FlatStepTypeMessagePubSubType(), data.getFlatStepType());

      ser.read_type_a("stairs_step_type", new controller_msgs.msg.dds.QuixStairsStepTypeMessagePubSubType(), data.getStairsStepType());

      ser.read_type_a("side_step_direction", new controller_msgs.msg.dds.QuixSideStepDirectionMessagePubSubType(), data.getSideStepDirection());

      ser.read_type_a("slope_step_type", new controller_msgs.msg.dds.QuixSlopeStepTypeMessagePubSubType(), data.getSlopeStepType());

      data.setForceSwingSide(ser.read_type_9("force_swing_side"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuixCrutchMessage src, controller_msgs.msg.dds.QuixCrutchMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuixCrutchMessage createData()
   {
      return new controller_msgs.msg.dds.QuixCrutchMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuixCrutchMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuixCrutchMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuixCrutchMessage src, controller_msgs.msg.dds.QuixCrutchMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuixCrutchMessagePubSubType newInstance()
   {
      return new QuixCrutchMessagePubSubType();
   }
}

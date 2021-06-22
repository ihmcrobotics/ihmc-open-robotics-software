package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PushRecoveryResultMessage" defined in "PushRecoveryResultMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PushRecoveryResultMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PushRecoveryResultMessage_.idl instead.
*
*/
public class PushRecoveryResultMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PushRecoveryResultMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PushRecoveryResultMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PushRecoveryResultMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PushRecoveryResultMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 5; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PushRecoveryResultMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PushRecoveryResultMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getRecoverySteps(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getStepConstraintList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.getCdrSerializedSize(data.getStepConstraintList().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PushRecoveryResultMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getRecoverySteps(), cdr);
      cdr.write_type_7(data.getIsStepRecoverable());

      if(data.getStepConstraintList().size() <= 5)
      cdr.write_type_e(data.getStepConstraintList());else
          throw new RuntimeException("step_constraint_list field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.PushRecoveryResultMessage data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getRecoverySteps(), cdr);	
      data.setIsStepRecoverable(cdr.read_type_7());
      	
      cdr.read_type_e(data.getStepConstraintList());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PushRecoveryResultMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("recovery_steps", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getRecoverySteps());

      ser.write_type_7("is_step_recoverable", data.getIsStepRecoverable());
      ser.write_type_e("step_constraint_list", data.getStepConstraintList());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PushRecoveryResultMessage data)
   {
      ser.read_type_a("recovery_steps", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getRecoverySteps());

      data.setIsStepRecoverable(ser.read_type_7("is_step_recoverable"));
      ser.read_type_e("step_constraint_list", data.getStepConstraintList());
   }

   public static void staticCopy(controller_msgs.msg.dds.PushRecoveryResultMessage src, controller_msgs.msg.dds.PushRecoveryResultMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PushRecoveryResultMessage createData()
   {
      return new controller_msgs.msg.dds.PushRecoveryResultMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PushRecoveryResultMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PushRecoveryResultMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PushRecoveryResultMessage src, controller_msgs.msg.dds.PushRecoveryResultMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PushRecoveryResultMessagePubSubType newInstance()
   {
      return new PushRecoveryResultMessagePubSubType();
   }
}

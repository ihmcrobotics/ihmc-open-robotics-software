package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ContinuousWalkingCommandMessage" defined in "ContinuousWalkingCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ContinuousWalkingCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ContinuousWalkingCommandMessage_.idl instead.
*
*/
public class ContinuousWalkingCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ContinuousWalkingCommandMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ContinuousWalkingCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0c997ddb04139b7ebbdc0f41ae14e427e5d37bd90365220adf611138fa40f6d6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getPublishToController());

      cdr.write_type_7(data.getEnableContinuousWalking());

      cdr.write_type_3(data.getNumberOfStepsToSend());

      cdr.write_type_6(data.getForwardValue());

      cdr.write_type_6(data.getLateralValue());

      cdr.write_type_6(data.getTurningValue());

   }

   public static void read(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setPublishToController(cdr.read_type_7());
      	
      data.setEnableContinuousWalking(cdr.read_type_7());
      	
      data.setNumberOfStepsToSend(cdr.read_type_3());
      	
      data.setForwardValue(cdr.read_type_6());
      	
      data.setLateralValue(cdr.read_type_6());
      	
      data.setTurningValue(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("publish_to_controller", data.getPublishToController());
      ser.write_type_7("enable_continuous_walking", data.getEnableContinuousWalking());
      ser.write_type_3("number_of_steps_to_send", data.getNumberOfStepsToSend());
      ser.write_type_6("forward_value", data.getForwardValue());
      ser.write_type_6("lateral_value", data.getLateralValue());
      ser.write_type_6("turning_value", data.getTurningValue());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data)
   {
      data.setPublishToController(ser.read_type_7("publish_to_controller"));
      data.setEnableContinuousWalking(ser.read_type_7("enable_continuous_walking"));
      data.setNumberOfStepsToSend(ser.read_type_3("number_of_steps_to_send"));
      data.setForwardValue(ser.read_type_6("forward_value"));
      data.setLateralValue(ser.read_type_6("lateral_value"));
      data.setTurningValue(ser.read_type_6("turning_value"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage src, behavior_msgs.msg.dds.ContinuousWalkingCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ContinuousWalkingCommandMessage createData()
   {
      return new behavior_msgs.msg.dds.ContinuousWalkingCommandMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ContinuousWalkingCommandMessage src, behavior_msgs.msg.dds.ContinuousWalkingCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ContinuousWalkingCommandMessagePubSubType newInstance()
   {
      return new ContinuousWalkingCommandMessagePubSubType();
   }
}

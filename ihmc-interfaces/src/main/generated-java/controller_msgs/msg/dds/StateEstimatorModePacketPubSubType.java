package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StateEstimatorModePacket" defined in "StateEstimatorModePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StateEstimatorModePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StateEstimatorModePacket_.idl instead.
*
*/
public class StateEstimatorModePacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.StateEstimatorModePacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::StateEstimatorModePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6d07cdee0553c23b74e3d9614a6ed76ebd4f2de15146adf9870dff3bea7245d3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.StateEstimatorModePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.StateEstimatorModePacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StateEstimatorModePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.StateEstimatorModePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.StateEstimatorModePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRequestedStateEstimatorMode());

   }

   public static void read(controller_msgs.msg.dds.StateEstimatorModePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRequestedStateEstimatorMode(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.StateEstimatorModePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("requested_state_estimator_mode", data.getRequestedStateEstimatorMode());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.StateEstimatorModePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRequestedStateEstimatorMode(ser.read_type_9("requested_state_estimator_mode"));
   }

   public static void staticCopy(controller_msgs.msg.dds.StateEstimatorModePacket src, controller_msgs.msg.dds.StateEstimatorModePacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.StateEstimatorModePacket createData()
   {
      return new controller_msgs.msg.dds.StateEstimatorModePacket();
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
   
   public void serialize(controller_msgs.msg.dds.StateEstimatorModePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.StateEstimatorModePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.StateEstimatorModePacket src, controller_msgs.msg.dds.StateEstimatorModePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StateEstimatorModePacketPubSubType newInstance()
   {
      return new StateEstimatorModePacketPubSubType();
   }
}

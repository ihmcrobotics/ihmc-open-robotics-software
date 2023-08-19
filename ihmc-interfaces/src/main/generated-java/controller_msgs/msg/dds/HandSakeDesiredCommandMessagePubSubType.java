package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandSakeDesiredCommandMessage" defined in "HandSakeDesiredCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandSakeDesiredCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandSakeDesiredCommandMessage_.idl instead.
*
*/
public class HandSakeDesiredCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandSakeDesiredCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandSakeDesiredCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "22305c68d3c88a32d98689c96452b8e810ebde4fc7b99b8a983bf213a3a6c39d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandSakeDesiredCommandMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_9(data.getDesiredHandConfiguration());

      cdr.write_type_6(data.getPostionRatio());

      cdr.write_type_6(data.getTorqueRatio());

   }

   public static void read(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setDesiredHandConfiguration(cdr.read_type_9());
      	
      data.setPostionRatio(cdr.read_type_6());
      	
      data.setTorqueRatio(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_9("desired_hand_configuration", data.getDesiredHandConfiguration());
      ser.write_type_6("postion_ratio", data.getPostionRatio());
      ser.write_type_6("torque_ratio", data.getTorqueRatio());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandSakeDesiredCommandMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setDesiredHandConfiguration(ser.read_type_9("desired_hand_configuration"));
      data.setPostionRatio(ser.read_type_6("postion_ratio"));
      data.setTorqueRatio(ser.read_type_6("torque_ratio"));
   }

   public static void staticCopy(controller_msgs.msg.dds.HandSakeDesiredCommandMessage src, controller_msgs.msg.dds.HandSakeDesiredCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandSakeDesiredCommandMessage createData()
   {
      return new controller_msgs.msg.dds.HandSakeDesiredCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandSakeDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandSakeDesiredCommandMessage src, controller_msgs.msg.dds.HandSakeDesiredCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandSakeDesiredCommandMessagePubSubType newInstance()
   {
      return new HandSakeDesiredCommandMessagePubSubType();
   }
}

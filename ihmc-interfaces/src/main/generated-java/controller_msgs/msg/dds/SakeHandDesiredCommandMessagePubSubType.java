package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SakeHandDesiredCommandMessage" defined in "SakeHandDesiredCommandMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SakeHandDesiredCommandMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SakeHandDesiredCommandMessage_.idl instead.
*
*/
public class SakeHandDesiredCommandMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SakeHandDesiredCommandMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SakeHandDesiredCommandMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f78813ecc39f58314d7b334c0e08265b3d2b19f06992f7be1ec117d81e8ab732";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SakeHandDesiredCommandMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_9(data.getDesiredHandConfiguration());

      cdr.write_type_6(data.getPostionRatio());

      cdr.write_type_6(data.getTorqueRatio());

   }

   public static void read(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setDesiredHandConfiguration(cdr.read_type_9());
      	
      data.setPostionRatio(cdr.read_type_6());
      	
      data.setTorqueRatio(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_9("desired_hand_configuration", data.getDesiredHandConfiguration());
      ser.write_type_6("postion_ratio", data.getPostionRatio());
      ser.write_type_6("torque_ratio", data.getTorqueRatio());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SakeHandDesiredCommandMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setDesiredHandConfiguration(ser.read_type_9("desired_hand_configuration"));
      data.setPostionRatio(ser.read_type_6("postion_ratio"));
      data.setTorqueRatio(ser.read_type_6("torque_ratio"));
   }

   public static void staticCopy(controller_msgs.msg.dds.SakeHandDesiredCommandMessage src, controller_msgs.msg.dds.SakeHandDesiredCommandMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SakeHandDesiredCommandMessage createData()
   {
      return new controller_msgs.msg.dds.SakeHandDesiredCommandMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SakeHandDesiredCommandMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SakeHandDesiredCommandMessage src, controller_msgs.msg.dds.SakeHandDesiredCommandMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SakeHandDesiredCommandMessagePubSubType newInstance()
   {
      return new SakeHandDesiredCommandMessagePubSubType();
   }
}

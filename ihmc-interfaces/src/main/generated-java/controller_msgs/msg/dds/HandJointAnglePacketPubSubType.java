package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandJointAnglePacket" defined in "HandJointAnglePacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandJointAnglePacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandJointAnglePacket_.idl instead.
*
*/
public class HandJointAnglePacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandJointAnglePacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandJointAnglePacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9d30fd1e55c53b9d100b83db7979799ac34651bb90fcd7132fed28c650559283";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandJointAnglePacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandJointAnglePacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandJointAnglePacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandJointAnglePacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointAngles().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandJointAnglePacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      if(data.getJointAngles().size() <= 100)
      cdr.write_type_e(data.getJointAngles());else
          throw new RuntimeException("joint_angles field exceeds the maximum length");

      cdr.write_type_7(data.getConnected());

      cdr.write_type_7(data.getCalibrated());

   }

   public static void read(controller_msgs.msg.dds.HandJointAnglePacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      cdr.read_type_e(data.getJointAngles());	
      data.setConnected(cdr.read_type_7());
      	
      data.setCalibrated(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandJointAnglePacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_e("joint_angles", data.getJointAngles());
      ser.write_type_7("connected", data.getConnected());
      ser.write_type_7("calibrated", data.getCalibrated());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandJointAnglePacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_e("joint_angles", data.getJointAngles());
      data.setConnected(ser.read_type_7("connected"));
      data.setCalibrated(ser.read_type_7("calibrated"));
   }

   public static void staticCopy(controller_msgs.msg.dds.HandJointAnglePacket src, controller_msgs.msg.dds.HandJointAnglePacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandJointAnglePacket createData()
   {
      return new controller_msgs.msg.dds.HandJointAnglePacket();
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
   
   public void serialize(controller_msgs.msg.dds.HandJointAnglePacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandJointAnglePacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandJointAnglePacket src, controller_msgs.msg.dds.HandJointAnglePacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandJointAnglePacketPubSubType newInstance()
   {
      return new HandJointAnglePacketPubSubType();
   }
}

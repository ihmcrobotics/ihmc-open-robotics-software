package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ArmDesiredAccelerationsMessage" defined in "ArmDesiredAccelerationsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ArmDesiredAccelerationsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ArmDesiredAccelerationsMessage_.idl instead.
*
*/
public class ArmDesiredAccelerationsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ArmDesiredAccelerationsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ArmDesiredAccelerationsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "592d45b54ba2badd9268fb234991f0e34fdb59adf811c69958f3d872e7a6ec4d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.getCdrSerializedSize(data.getDesiredAccelerations(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.write(data.getDesiredAccelerations(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.read(data.getDesiredAccelerations(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());

   }

   public static void staticCopy(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage src, controller_msgs.msg.dds.ArmDesiredAccelerationsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ArmDesiredAccelerationsMessage createData()
   {
      return new controller_msgs.msg.dds.ArmDesiredAccelerationsMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ArmDesiredAccelerationsMessage src, controller_msgs.msg.dds.ArmDesiredAccelerationsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ArmDesiredAccelerationsMessagePubSubType newInstance()
   {
      return new ArmDesiredAccelerationsMessagePubSubType();
   }
}

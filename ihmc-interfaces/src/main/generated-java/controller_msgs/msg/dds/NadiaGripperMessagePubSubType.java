package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "NadiaGripperMessage" defined in "NadiaGripperMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from NadiaGripperMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit NadiaGripperMessage_.idl instead.
*
*/
public class NadiaGripperMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.NadiaGripperMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::NadiaGripperMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.NadiaGripperMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.NadiaGripperMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NadiaGripperMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NadiaGripperMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.NadiaGripperMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getCommand());

      cdr.write_type_6(data.getDesiredTorqueRatio());

      cdr.write_type_6(data.getDesiredPositionRatio());

      cdr.write_type_6(data.getMeasuredPositionRatio());

   }

   public static void read(controller_msgs.msg.dds.NadiaGripperMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setCommand(cdr.read_type_9());
      	
      data.setDesiredTorqueRatio(cdr.read_type_6());
      	
      data.setDesiredPositionRatio(cdr.read_type_6());
      	
      data.setMeasuredPositionRatio(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.NadiaGripperMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("command", data.getCommand());
      ser.write_type_6("desired_torque_ratio", data.getDesiredTorqueRatio());
      ser.write_type_6("desired_position_ratio", data.getDesiredPositionRatio());
      ser.write_type_6("measured_position_ratio", data.getMeasuredPositionRatio());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.NadiaGripperMessage data)
   {
      data.setCommand(ser.read_type_9("command"));
      data.setDesiredTorqueRatio(ser.read_type_6("desired_torque_ratio"));
      data.setDesiredPositionRatio(ser.read_type_6("desired_position_ratio"));
      data.setMeasuredPositionRatio(ser.read_type_6("measured_position_ratio"));
   }

   public static void staticCopy(controller_msgs.msg.dds.NadiaGripperMessage src, controller_msgs.msg.dds.NadiaGripperMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.NadiaGripperMessage createData()
   {
      return new controller_msgs.msg.dds.NadiaGripperMessage();
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
   
   public void serialize(controller_msgs.msg.dds.NadiaGripperMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.NadiaGripperMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.NadiaGripperMessage src, controller_msgs.msg.dds.NadiaGripperMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public NadiaGripperMessagePubSubType newInstance()
   {
      return new NadiaGripperMessagePubSubType();
   }
}

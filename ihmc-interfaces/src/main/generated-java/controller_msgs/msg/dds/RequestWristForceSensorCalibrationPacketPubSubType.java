package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RequestWristForceSensorCalibrationPacket" defined in "RequestWristForceSensorCalibrationPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RequestWristForceSensorCalibrationPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RequestWristForceSensorCalibrationPacket_.idl instead.
*
*/
public class RequestWristForceSensorCalibrationPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RequestWristForceSensorCalibrationPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "1c1fe63a0b97f3f9fe2d46358e08184197a53c54d7f989ecc4ad4ed60cd7adee";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

   }

   public static void read(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));   }

   public static void staticCopy(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket src, controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket createData()
   {
      return new controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket();
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
   
   public void serialize(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket src, controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RequestWristForceSensorCalibrationPacketPubSubType newInstance()
   {
      return new RequestWristForceSensorCalibrationPacketPubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DrillDetectionPacket" defined in "DrillDetectionPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DrillDetectionPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DrillDetectionPacket_.idl instead.
*
*/
public class DrillDetectionPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DrillDetectionPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DrillDetectionPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DrillDetectionPacket data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DrillDetectionPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DrillDetectionPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_7(data.getIsDrillOn());

   }

   public static void read(controller_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setIsDrillOn(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_7("is_drill_on", data.getIsDrillOn());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DrillDetectionPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setIsDrillOn(ser.read_type_7("is_drill_on"));
   }

   public static void staticCopy(controller_msgs.msg.dds.DrillDetectionPacket src, controller_msgs.msg.dds.DrillDetectionPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.DrillDetectionPacket createData()
   {
      return new controller_msgs.msg.dds.DrillDetectionPacket();
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
   
   public void serialize(controller_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.DrillDetectionPacket src, controller_msgs.msg.dds.DrillDetectionPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DrillDetectionPacketPubSubType newInstance()
   {
      return new DrillDetectionPacketPubSubType();
   }
}

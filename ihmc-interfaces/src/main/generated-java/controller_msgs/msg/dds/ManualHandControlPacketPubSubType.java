package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ManualHandControlPacket" defined in "ManualHandControlPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ManualHandControlPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ManualHandControlPacket_.idl instead.
*
*/
public class ManualHandControlPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ManualHandControlPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ManualHandControlPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d732352a47b4ed06efd720b01295c5d1c9997be09abebcc90f5fa4e54f288af1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ManualHandControlPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ManualHandControlPacket data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ManualHandControlPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ManualHandControlPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ManualHandControlPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_6(data.getIndex());

      cdr.write_type_6(data.getMiddle());

      cdr.write_type_6(data.getThumb());

      cdr.write_type_6(data.getSpread());

      cdr.write_type_2(data.getControlType());

   }

   public static void read(controller_msgs.msg.dds.ManualHandControlPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setIndex(cdr.read_type_6());
      	
      data.setMiddle(cdr.read_type_6());
      	
      data.setThumb(cdr.read_type_6());
      	
      data.setSpread(cdr.read_type_6());
      	
      data.setControlType(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ManualHandControlPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_6("index", data.getIndex());
      ser.write_type_6("middle", data.getMiddle());
      ser.write_type_6("thumb", data.getThumb());
      ser.write_type_6("spread", data.getSpread());
      ser.write_type_2("control_type", data.getControlType());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ManualHandControlPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setIndex(ser.read_type_6("index"));
      data.setMiddle(ser.read_type_6("middle"));
      data.setThumb(ser.read_type_6("thumb"));
      data.setSpread(ser.read_type_6("spread"));
      data.setControlType(ser.read_type_2("control_type"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ManualHandControlPacket src, controller_msgs.msg.dds.ManualHandControlPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ManualHandControlPacket createData()
   {
      return new controller_msgs.msg.dds.ManualHandControlPacket();
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
   
   public void serialize(controller_msgs.msg.dds.ManualHandControlPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ManualHandControlPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ManualHandControlPacket src, controller_msgs.msg.dds.ManualHandControlPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ManualHandControlPacketPubSubType newInstance()
   {
      return new ManualHandControlPacketPubSubType();
   }
}

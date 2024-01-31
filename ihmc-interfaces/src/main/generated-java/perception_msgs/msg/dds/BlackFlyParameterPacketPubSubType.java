package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BlackFlyParameterPacket" defined in "BlackFlyParameterPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BlackFlyParameterPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BlackFlyParameterPacket_.idl instead.
*
*/
public class BlackFlyParameterPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.BlackFlyParameterPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::BlackFlyParameterPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "756cfc09d296ba1a816954b61fb0673000f3b15f9e1a8981c53baef0ae4ba9ee";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.BlackFlyParameterPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.BlackFlyParameterPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.BlackFlyParameterPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.BlackFlyParameterPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.BlackFlyParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getAutoExposure());

      cdr.write_type_7(data.getAutoGain());

      cdr.write_type_7(data.getAutoShutter());

      cdr.write_type_6(data.getExposure());

      cdr.write_type_6(data.getFrameRate());

      cdr.write_type_7(data.getFromUi());

      cdr.write_type_6(data.getGain());

      cdr.write_type_6(data.getShutter());

      cdr.write_type_9(data.getRobotSide());

   }

   public static void read(perception_msgs.msg.dds.BlackFlyParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setAutoExposure(cdr.read_type_7());
      	
      data.setAutoGain(cdr.read_type_7());
      	
      data.setAutoShutter(cdr.read_type_7());
      	
      data.setExposure(cdr.read_type_6());
      	
      data.setFrameRate(cdr.read_type_6());
      	
      data.setFromUi(cdr.read_type_7());
      	
      data.setGain(cdr.read_type_6());
      	
      data.setShutter(cdr.read_type_6());
      	
      data.setRobotSide(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.BlackFlyParameterPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("auto_exposure", data.getAutoExposure());
      ser.write_type_7("auto_gain", data.getAutoGain());
      ser.write_type_7("auto_shutter", data.getAutoShutter());
      ser.write_type_6("exposure", data.getExposure());
      ser.write_type_6("frame_rate", data.getFrameRate());
      ser.write_type_7("from_ui", data.getFromUi());
      ser.write_type_6("gain", data.getGain());
      ser.write_type_6("shutter", data.getShutter());
      ser.write_type_9("robot_side", data.getRobotSide());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.BlackFlyParameterPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setAutoExposure(ser.read_type_7("auto_exposure"));
      data.setAutoGain(ser.read_type_7("auto_gain"));
      data.setAutoShutter(ser.read_type_7("auto_shutter"));
      data.setExposure(ser.read_type_6("exposure"));
      data.setFrameRate(ser.read_type_6("frame_rate"));
      data.setFromUi(ser.read_type_7("from_ui"));
      data.setGain(ser.read_type_6("gain"));
      data.setShutter(ser.read_type_6("shutter"));
      data.setRobotSide(ser.read_type_9("robot_side"));
   }

   public static void staticCopy(perception_msgs.msg.dds.BlackFlyParameterPacket src, perception_msgs.msg.dds.BlackFlyParameterPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.BlackFlyParameterPacket createData()
   {
      return new perception_msgs.msg.dds.BlackFlyParameterPacket();
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
   
   public void serialize(perception_msgs.msg.dds.BlackFlyParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.BlackFlyParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.BlackFlyParameterPacket src, perception_msgs.msg.dds.BlackFlyParameterPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BlackFlyParameterPacketPubSubType newInstance()
   {
      return new BlackFlyParameterPacketPubSubType();
   }
}

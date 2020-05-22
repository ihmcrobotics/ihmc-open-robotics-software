package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultisenseParameterPacket" defined in "MultisenseParameterPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultisenseParameterPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultisenseParameterPacket_.idl instead.
*
*/
public class MultisenseParameterPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.MultisenseParameterPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::MultisenseParameterPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.MultisenseParameterPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.MultisenseParameterPacket data) throws java.io.IOException
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


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultisenseParameterPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultisenseParameterPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.MultisenseParameterPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_7(data.getInitialize());


      cdr.write_type_6(data.getGain());


      cdr.write_type_6(data.getMotorSpeed());


      cdr.write_type_7(data.getLedEnable());


      cdr.write_type_7(data.getFlashEnable());


      cdr.write_type_6(data.getDutyCycle());


      cdr.write_type_7(data.getAutoExposure());


      cdr.write_type_7(data.getAutoWhiteBalance());

   }

   public static void read(controller_msgs.msg.dds.MultisenseParameterPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setInitialize(cdr.read_type_7());
      	

      data.setGain(cdr.read_type_6());
      	

      data.setMotorSpeed(cdr.read_type_6());
      	

      data.setLedEnable(cdr.read_type_7());
      	

      data.setFlashEnable(cdr.read_type_7());
      	

      data.setDutyCycle(cdr.read_type_6());
      	

      data.setAutoExposure(cdr.read_type_7());
      	

      data.setAutoWhiteBalance(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.MultisenseParameterPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_7("initialize", data.getInitialize());

      ser.write_type_6("gain", data.getGain());

      ser.write_type_6("motor_speed", data.getMotorSpeed());

      ser.write_type_7("led_enable", data.getLedEnable());

      ser.write_type_7("flash_enable", data.getFlashEnable());

      ser.write_type_6("duty_cycle", data.getDutyCycle());

      ser.write_type_7("auto_exposure", data.getAutoExposure());

      ser.write_type_7("auto_white_balance", data.getAutoWhiteBalance());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.MultisenseParameterPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setInitialize(ser.read_type_7("initialize"));

      data.setGain(ser.read_type_6("gain"));

      data.setMotorSpeed(ser.read_type_6("motor_speed"));

      data.setLedEnable(ser.read_type_7("led_enable"));

      data.setFlashEnable(ser.read_type_7("flash_enable"));

      data.setDutyCycle(ser.read_type_6("duty_cycle"));

      data.setAutoExposure(ser.read_type_7("auto_exposure"));

      data.setAutoWhiteBalance(ser.read_type_7("auto_white_balance"));
   }

   public static void staticCopy(controller_msgs.msg.dds.MultisenseParameterPacket src, controller_msgs.msg.dds.MultisenseParameterPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.MultisenseParameterPacket createData()
   {
      return new controller_msgs.msg.dds.MultisenseParameterPacket();
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
   
   public void serialize(controller_msgs.msg.dds.MultisenseParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.MultisenseParameterPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.MultisenseParameterPacket src, controller_msgs.msg.dds.MultisenseParameterPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultisenseParameterPacketPubSubType newInstance()
   {
      return new MultisenseParameterPacketPubSubType();
   }
}

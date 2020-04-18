package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TextToSpeechPacket" defined in "TextToSpeechPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TextToSpeechPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TextToSpeechPacket_.idl instead.
*
*/
public class TextToSpeechPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.TextToSpeechPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::TextToSpeechPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.TextToSpeechPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.TextToSpeechPacket data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TextToSpeechPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TextToSpeechPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getTextToSpeak().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.TextToSpeechPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_7(data.getSpeakPacket());


      cdr.write_type_7(data.getBeep());


      if(data.getTextToSpeak().length() <= 255)
      cdr.write_type_d(data.getTextToSpeak());else
          throw new RuntimeException("text_to_speak field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.TextToSpeechPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setSpeakPacket(cdr.read_type_7());
      	

      data.setBeep(cdr.read_type_7());
      	

      cdr.read_type_d(data.getTextToSpeak());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.TextToSpeechPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_7("speak_packet", data.getSpeakPacket());

      ser.write_type_7("beep", data.getBeep());

      ser.write_type_d("text_to_speak", data.getTextToSpeak());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.TextToSpeechPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setSpeakPacket(ser.read_type_7("speak_packet"));

      data.setBeep(ser.read_type_7("beep"));

      ser.read_type_d("text_to_speak", data.getTextToSpeak());
   }

   public static void staticCopy(controller_msgs.msg.dds.TextToSpeechPacket src, controller_msgs.msg.dds.TextToSpeechPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.TextToSpeechPacket createData()
   {
      return new controller_msgs.msg.dds.TextToSpeechPacket();
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
   
   public void serialize(controller_msgs.msg.dds.TextToSpeechPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.TextToSpeechPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.TextToSpeechPacket src, controller_msgs.msg.dds.TextToSpeechPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TextToSpeechPacketPubSubType newInstance()
   {
      return new TextToSpeechPacketPubSubType();
   }
}

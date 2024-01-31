package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TreadmillMessage" defined in "TreadmillMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TreadmillMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TreadmillMessage_.idl instead.
*
*/
public class TreadmillMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.TreadmillMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::TreadmillMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7564a0c4ced1b9dc6410974ae416190ea1f5f0980f57b837c3b7ef7c77ddbb28";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.TreadmillMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.TreadmillMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.TreadmillMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.TreadmillMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.TreadmillMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getActivate());

      cdr.write_type_9(data.getAction());

      cdr.write_type_6(data.getSpeed());

      cdr.write_type_6(data.getIncline());

   }

   public static void read(exoskeleton_msgs.msg.dds.TreadmillMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setActivate(cdr.read_type_7());
      	
      data.setAction(cdr.read_type_9());
      	
      data.setSpeed(cdr.read_type_6());
      	
      data.setIncline(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.TreadmillMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("activate", data.getActivate());
      ser.write_type_9("action", data.getAction());
      ser.write_type_6("speed", data.getSpeed());
      ser.write_type_6("incline", data.getIncline());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.TreadmillMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setActivate(ser.read_type_7("activate"));
      data.setAction(ser.read_type_9("action"));
      data.setSpeed(ser.read_type_6("speed"));
      data.setIncline(ser.read_type_6("incline"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.TreadmillMessage src, exoskeleton_msgs.msg.dds.TreadmillMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.TreadmillMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.TreadmillMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.TreadmillMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.TreadmillMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.TreadmillMessage src, exoskeleton_msgs.msg.dds.TreadmillMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TreadmillMessagePubSubType newInstance()
   {
      return new TreadmillMessagePubSubType();
   }
}

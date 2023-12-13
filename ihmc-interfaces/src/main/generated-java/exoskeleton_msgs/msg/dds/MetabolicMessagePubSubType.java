package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MetabolicMessage" defined in "MetabolicMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MetabolicMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MetabolicMessage_.idl instead.
*
*/
public class MetabolicMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.MetabolicMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::MetabolicMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "844d3023552eb76a56fcb2bd7d13d0931c0fa125faa46e39fce01bac859728dd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.MetabolicMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.MetabolicMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.MetabolicMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.MetabolicMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.MetabolicMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getMetabolicRate());

      cdr.write_type_6(data.getDeltaMetabolicRate());

   }

   public static void read(exoskeleton_msgs.msg.dds.MetabolicMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setMetabolicRate(cdr.read_type_6());
      	
      data.setDeltaMetabolicRate(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.MetabolicMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("metabolic_rate", data.getMetabolicRate());
      ser.write_type_6("delta_metabolic_rate", data.getDeltaMetabolicRate());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.MetabolicMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setMetabolicRate(ser.read_type_6("metabolic_rate"));
      data.setDeltaMetabolicRate(ser.read_type_6("delta_metabolic_rate"));
   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.MetabolicMessage src, exoskeleton_msgs.msg.dds.MetabolicMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.MetabolicMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.MetabolicMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.MetabolicMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.MetabolicMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.MetabolicMessage src, exoskeleton_msgs.msg.dds.MetabolicMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MetabolicMessagePubSubType newInstance()
   {
      return new MetabolicMessagePubSubType();
   }
}

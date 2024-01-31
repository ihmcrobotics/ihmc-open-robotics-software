package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LocalizationPointMapPacket" defined in "LocalizationPointMapPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LocalizationPointMapPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LocalizationPointMapPacket_.idl instead.
*
*/
public class LocalizationPointMapPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.LocalizationPointMapPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::LocalizationPointMapPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9a578ef99b4ca95858e1b158a5c5668f49282569f40f3b4cc54b74ac008b00ce";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.LocalizationPointMapPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.LocalizationPointMapPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LocalizationPointMapPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LocalizationPointMapPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getLocalizationPointMap().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.LocalizationPointMapPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      if(data.getLocalizationPointMap().size() <= 100)
      cdr.write_type_e(data.getLocalizationPointMap());else
          throw new RuntimeException("localization_point_map field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.LocalizationPointMapPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      cdr.read_type_e(data.getLocalizationPointMap());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.LocalizationPointMapPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_e("localization_point_map", data.getLocalizationPointMap());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.LocalizationPointMapPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      ser.read_type_e("localization_point_map", data.getLocalizationPointMap());
   }

   public static void staticCopy(controller_msgs.msg.dds.LocalizationPointMapPacket src, controller_msgs.msg.dds.LocalizationPointMapPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.LocalizationPointMapPacket createData()
   {
      return new controller_msgs.msg.dds.LocalizationPointMapPacket();
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
   
   public void serialize(controller_msgs.msg.dds.LocalizationPointMapPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.LocalizationPointMapPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.LocalizationPointMapPacket src, controller_msgs.msg.dds.LocalizationPointMapPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LocalizationPointMapPacketPubSubType newInstance()
   {
      return new LocalizationPointMapPacketPubSubType();
   }
}

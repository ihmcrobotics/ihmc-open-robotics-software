package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeatMapPacket" defined in "HeatMapPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeatMapPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeatMapPacket_.idl instead.
*
*/
public class HeatMapPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.HeatMapPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::HeatMapPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "05ab32112d61ad896101bc0afd442da65fdd3e1832612f077d3a9eb5baaec3b5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.HeatMapPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.HeatMapPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeatMapPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeatMapPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.HeatMapPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getData().size() <= 100)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

      cdr.write_type_2(data.getWidth());

      cdr.write_type_2(data.getHeight());

      if(data.getName().length() <= 255)
      cdr.write_type_d(data.getName());else
          throw new RuntimeException("name field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.HeatMapPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getData());	
      data.setWidth(cdr.read_type_2());
      	
      data.setHeight(cdr.read_type_2());
      	
      cdr.read_type_d(data.getName());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.HeatMapPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("data", data.getData());
      ser.write_type_2("width", data.getWidth());
      ser.write_type_2("height", data.getHeight());
      ser.write_type_d("name", data.getName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.HeatMapPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("data", data.getData());
      data.setWidth(ser.read_type_2("width"));
      data.setHeight(ser.read_type_2("height"));
      ser.read_type_d("name", data.getName());
   }

   public static void staticCopy(perception_msgs.msg.dds.HeatMapPacket src, perception_msgs.msg.dds.HeatMapPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.HeatMapPacket createData()
   {
      return new perception_msgs.msg.dds.HeatMapPacket();
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
   
   public void serialize(perception_msgs.msg.dds.HeatMapPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.HeatMapPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.HeatMapPacket src, perception_msgs.msg.dds.HeatMapPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeatMapPacketPubSubType newInstance()
   {
      return new HeatMapPacketPubSubType();
   }
}

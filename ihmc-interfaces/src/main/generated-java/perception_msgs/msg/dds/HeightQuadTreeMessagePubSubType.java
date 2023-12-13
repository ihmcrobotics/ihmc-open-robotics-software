package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightQuadTreeMessage" defined in "HeightQuadTreeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightQuadTreeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightQuadTreeMessage_.idl instead.
*
*/
public class HeightQuadTreeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.HeightQuadTreeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::HeightQuadTreeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c5d5273afb994c5ea2860a5fbcd25182a1c3b101bf7a16a685f0852b5b82a7ce";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.HeightQuadTreeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.HeightQuadTreeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 5000; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.HeightQuadTreeLeafMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightQuadTreeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightQuadTreeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLeaves().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.HeightQuadTreeLeafMessagePubSubType.getCdrSerializedSize(data.getLeaves().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.HeightQuadTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_5(data.getDefaultHeight());

      cdr.write_type_5(data.getResolution());

      cdr.write_type_5(data.getSizeX());

      cdr.write_type_5(data.getSizeY());

      if(data.getLeaves().size() <= 5000)
      cdr.write_type_e(data.getLeaves());else
          throw new RuntimeException("leaves field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.HeightQuadTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setDefaultHeight(cdr.read_type_5());
      	
      data.setResolution(cdr.read_type_5());
      	
      data.setSizeX(cdr.read_type_5());
      	
      data.setSizeY(cdr.read_type_5());
      	
      cdr.read_type_e(data.getLeaves());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.HeightQuadTreeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_5("default_height", data.getDefaultHeight());
      ser.write_type_5("resolution", data.getResolution());
      ser.write_type_5("size_x", data.getSizeX());
      ser.write_type_5("size_y", data.getSizeY());
      ser.write_type_e("leaves", data.getLeaves());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.HeightQuadTreeMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setDefaultHeight(ser.read_type_5("default_height"));
      data.setResolution(ser.read_type_5("resolution"));
      data.setSizeX(ser.read_type_5("size_x"));
      data.setSizeY(ser.read_type_5("size_y"));
      ser.read_type_e("leaves", data.getLeaves());
   }

   public static void staticCopy(perception_msgs.msg.dds.HeightQuadTreeMessage src, perception_msgs.msg.dds.HeightQuadTreeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.HeightQuadTreeMessage createData()
   {
      return new perception_msgs.msg.dds.HeightQuadTreeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.HeightQuadTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.HeightQuadTreeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.HeightQuadTreeMessage src, perception_msgs.msg.dds.HeightQuadTreeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightQuadTreeMessagePubSubType newInstance()
   {
      return new HeightQuadTreeMessagePubSubType();
   }
}

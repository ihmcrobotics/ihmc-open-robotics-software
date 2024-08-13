package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GlobalMapTileMessage" defined in "GlobalMapTileMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GlobalMapTileMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GlobalMapTileMessage_.idl instead.
*
*/
public class GlobalMapTileMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.GlobalMapTileMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::GlobalMapTileMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6d19a28f0830a716c62a96cf86e2cbf89830dc93735f951814727e970d92a2f7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.GlobalMapTileMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.GlobalMapTileMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapTileMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapTileMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getCdrSerializedSize(data.getHeightMap(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.GlobalMapTileMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getCenterX());

      cdr.write_type_2(data.getCenterY());

      cdr.write_type_2(data.getHashCodeOfTile());

      perception_msgs.msg.dds.HeightMapMessagePubSubType.write(data.getHeightMap(), cdr);
   }

   public static void read(perception_msgs.msg.dds.GlobalMapTileMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setCenterX(cdr.read_type_2());
      	
      data.setCenterY(cdr.read_type_2());
      	
      data.setHashCodeOfTile(cdr.read_type_2());
      	
      perception_msgs.msg.dds.HeightMapMessagePubSubType.read(data.getHeightMap(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.GlobalMapTileMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("center_x", data.getCenterX());
      ser.write_type_2("center_y", data.getCenterY());
      ser.write_type_2("hash_code_of_tile", data.getHashCodeOfTile());
      ser.write_type_a("height_map", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMap());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.GlobalMapTileMessage data)
   {
      data.setCenterX(ser.read_type_2("center_x"));
      data.setCenterY(ser.read_type_2("center_y"));
      data.setHashCodeOfTile(ser.read_type_2("hash_code_of_tile"));
      ser.read_type_a("height_map", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMap());

   }

   public static void staticCopy(perception_msgs.msg.dds.GlobalMapTileMessage src, perception_msgs.msg.dds.GlobalMapTileMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.GlobalMapTileMessage createData()
   {
      return new perception_msgs.msg.dds.GlobalMapTileMessage();
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
   
   public void serialize(perception_msgs.msg.dds.GlobalMapTileMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.GlobalMapTileMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.GlobalMapTileMessage src, perception_msgs.msg.dds.GlobalMapTileMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GlobalMapTileMessagePubSubType newInstance()
   {
      return new GlobalMapTileMessagePubSubType();
   }
}

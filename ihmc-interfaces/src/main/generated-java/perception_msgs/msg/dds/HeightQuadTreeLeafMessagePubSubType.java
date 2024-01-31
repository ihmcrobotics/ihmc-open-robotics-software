package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightQuadTreeLeafMessage" defined in "HeightQuadTreeLeafMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightQuadTreeLeafMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightQuadTreeLeafMessage_.idl instead.
*
*/
public class HeightQuadTreeLeafMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.HeightQuadTreeLeafMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::HeightQuadTreeLeafMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ffcc8b5cfbccf04d80babba6b3d776c584a4687a6ceee2f300cbcd9b98987621";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.HeightQuadTreeLeafMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_5(data.getCenterX());

      cdr.write_type_5(data.getCenterY());

      cdr.write_type_5(data.getHeight());

   }

   public static void read(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCenterX(cdr.read_type_5());
      	
      data.setCenterY(cdr.read_type_5());
      	
      data.setHeight(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_5("center_x", data.getCenterX());
      ser.write_type_5("center_y", data.getCenterY());
      ser.write_type_5("height", data.getHeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.HeightQuadTreeLeafMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCenterX(ser.read_type_5("center_x"));
      data.setCenterY(ser.read_type_5("center_y"));
      data.setHeight(ser.read_type_5("height"));
   }

   public static void staticCopy(perception_msgs.msg.dds.HeightQuadTreeLeafMessage src, perception_msgs.msg.dds.HeightQuadTreeLeafMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.HeightQuadTreeLeafMessage createData()
   {
      return new perception_msgs.msg.dds.HeightQuadTreeLeafMessage();
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
   
   public void serialize(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.HeightQuadTreeLeafMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.HeightQuadTreeLeafMessage src, perception_msgs.msg.dds.HeightQuadTreeLeafMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightQuadTreeLeafMessagePubSubType newInstance()
   {
      return new HeightQuadTreeLeafMessagePubSubType();
   }
}

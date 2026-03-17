package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MeshSelectorBatchResult" defined in "MeshSelectorBatchResult_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MeshSelectorBatchResult_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MeshSelectorBatchResult_.idl instead.
*
*/
public class MeshSelectorBatchResultPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.MeshSelectorBatchResult>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::MeshSelectorBatchResult_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5a59ff487d066c4e3f9d1443aa918357d9d58f139f2c1edb87862835c2dae8f4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.MeshSelectorBatchResult data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.MeshSelectorBatchResult data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.MeshSelectorBatchResultItemPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.MeshSelectorBatchResult data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.MeshSelectorBatchResult data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getItems().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.MeshSelectorBatchResultItemPubSubType.getCdrSerializedSize(data.getItems().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.MeshSelectorBatchResult data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getFrameId());

      if(data.getItems().size() <= 100)
      cdr.write_type_e(data.getItems());else
          throw new RuntimeException("items field exceeds the maximum length: %d > %d".formatted(data.getItems().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.MeshSelectorBatchResult data, us.ihmc.idl.CDR cdr)
   {
      data.setFrameId(cdr.read_type_11());
      	
      cdr.read_type_e(data.getItems());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.MeshSelectorBatchResult data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("frame_id", data.getFrameId());
      ser.write_type_e("items", data.getItems());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.MeshSelectorBatchResult data)
   {
      data.setFrameId(ser.read_type_11("frame_id"));
      ser.read_type_e("items", data.getItems());
   }

   public static void staticCopy(perception_msgs.msg.dds.MeshSelectorBatchResult src, perception_msgs.msg.dds.MeshSelectorBatchResult dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.MeshSelectorBatchResult createData()
   {
      return new perception_msgs.msg.dds.MeshSelectorBatchResult();
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
   
   public void serialize(perception_msgs.msg.dds.MeshSelectorBatchResult data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.MeshSelectorBatchResult data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.MeshSelectorBatchResult src, perception_msgs.msg.dds.MeshSelectorBatchResult dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MeshSelectorBatchResultPubSubType newInstance()
   {
      return new MeshSelectorBatchResultPubSubType();
   }
}

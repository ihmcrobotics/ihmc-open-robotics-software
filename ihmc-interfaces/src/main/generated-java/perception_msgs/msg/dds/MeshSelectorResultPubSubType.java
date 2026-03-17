package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MeshSelectorResult" defined in "MeshSelectorResult_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MeshSelectorResult_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MeshSelectorResult_.idl instead.
*
*/
public class MeshSelectorResultPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.MeshSelectorResult>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::MeshSelectorResult_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "81c4305d79d17e1b5e94b653b6fc69ac0a890b4dc8836445424499129d89d2ee";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.MeshSelectorResult data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.MeshSelectorResult data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.MeshSelectorResult data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.MeshSelectorResult data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getCategory().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSelectedInstance().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.MeshSelectorResult data, us.ihmc.idl.CDR cdr)
   {
      if(data.getCategory().length() <= 255)
      cdr.write_type_d(data.getCategory());else
          throw new RuntimeException("category field exceeds the maximum length: %d > %d".formatted(data.getCategory().length(), 255));

      cdr.write_type_2(data.getTrackId());

      if(data.getSelectedInstance().length() <= 255)
      cdr.write_type_d(data.getSelectedInstance());else
          throw new RuntimeException("selected_instance field exceeds the maximum length: %d > %d".formatted(data.getSelectedInstance().length(), 255));

      cdr.write_type_5(data.getScore());

   }

   public static void read(perception_msgs.msg.dds.MeshSelectorResult data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getCategory());	
      data.setTrackId(cdr.read_type_2());
      	
      cdr.read_type_d(data.getSelectedInstance());	
      data.setScore(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.MeshSelectorResult data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("category", data.getCategory());
      ser.write_type_2("track_id", data.getTrackId());
      ser.write_type_d("selected_instance", data.getSelectedInstance());
      ser.write_type_5("score", data.getScore());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.MeshSelectorResult data)
   {
      ser.read_type_d("category", data.getCategory());
      data.setTrackId(ser.read_type_2("track_id"));
      ser.read_type_d("selected_instance", data.getSelectedInstance());
      data.setScore(ser.read_type_5("score"));
   }

   public static void staticCopy(perception_msgs.msg.dds.MeshSelectorResult src, perception_msgs.msg.dds.MeshSelectorResult dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.MeshSelectorResult createData()
   {
      return new perception_msgs.msg.dds.MeshSelectorResult();
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
   
   public void serialize(perception_msgs.msg.dds.MeshSelectorResult data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.MeshSelectorResult data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.MeshSelectorResult src, perception_msgs.msg.dds.MeshSelectorResult dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MeshSelectorResultPubSubType newInstance()
   {
      return new MeshSelectorResultPubSubType();
   }
}

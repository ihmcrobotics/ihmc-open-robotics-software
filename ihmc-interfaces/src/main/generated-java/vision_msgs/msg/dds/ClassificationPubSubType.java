package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Classification" defined in "Classification_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Classification_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Classification_.idl instead.
*
*/
public class ClassificationPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.Classification>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::Classification_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9c4361dd2393caabf112fda7a13d9e6754c0629a5da987b8f22e461a37c54434";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.Classification data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.Classification data) throws java.io.IOException
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

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += vision_msgs.msg.dds.ObjectHypothesisPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Classification data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.Classification data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getResults().size(); ++i0)
      {
          current_alignment += vision_msgs.msg.dds.ObjectHypothesisPubSubType.getCdrSerializedSize(data.getResults().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.Classification data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getResults().size() <= 100)
      cdr.write_type_e(data.getResults());else
          throw new RuntimeException("results field exceeds the maximum length: %d > %d".formatted(data.getResults().size(), 100));

   }

   public static void read(vision_msgs.msg.dds.Classification data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_e(data.getResults());	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.Classification data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("results", data.getResults());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.Classification data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("results", data.getResults());
   }

   public static void staticCopy(vision_msgs.msg.dds.Classification src, vision_msgs.msg.dds.Classification dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.Classification createData()
   {
      return new vision_msgs.msg.dds.Classification();
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
   
   public void serialize(vision_msgs.msg.dds.Classification data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.Classification data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.Classification src, vision_msgs.msg.dds.Classification dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ClassificationPubSubType newInstance()
   {
      return new ClassificationPubSubType();
   }
}

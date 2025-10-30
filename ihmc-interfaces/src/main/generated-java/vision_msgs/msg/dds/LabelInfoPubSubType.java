package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LabelInfo" defined in "LabelInfo_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LabelInfo_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LabelInfo_.idl instead.
*
*/
public class LabelInfoPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.LabelInfo>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::LabelInfo_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "749d9e052e35c705cac2ebabbfdbdf3b12650157be9fb3fa08912811b107cf9b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.LabelInfo data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.LabelInfo data) throws java.io.IOException
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
          current_alignment += vision_msgs.msg.dds.VisionClassPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.LabelInfo data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.LabelInfo data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getClassMap().size(); ++i0)
      {
          current_alignment += vision_msgs.msg.dds.VisionClassPubSubType.getCdrSerializedSize(data.getClassMap().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.LabelInfo data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getClassMap().size() <= 100)
      cdr.write_type_e(data.getClassMap());else
          throw new RuntimeException("class_map field exceeds the maximum length: %d > %d".formatted(data.getClassMap().size(), 100));

      cdr.write_type_5(data.getThreshold());

   }

   public static void read(vision_msgs.msg.dds.LabelInfo data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_e(data.getClassMap());	
      data.setThreshold(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.LabelInfo data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("class_map", data.getClassMap());
      ser.write_type_5("threshold", data.getThreshold());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.LabelInfo data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("class_map", data.getClassMap());
      data.setThreshold(ser.read_type_5("threshold"));
   }

   public static void staticCopy(vision_msgs.msg.dds.LabelInfo src, vision_msgs.msg.dds.LabelInfo dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.LabelInfo createData()
   {
      return new vision_msgs.msg.dds.LabelInfo();
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
   
   public void serialize(vision_msgs.msg.dds.LabelInfo data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.LabelInfo data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.LabelInfo src, vision_msgs.msg.dds.LabelInfo dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LabelInfoPubSubType newInstance()
   {
      return new LabelInfoPubSubType();
   }
}

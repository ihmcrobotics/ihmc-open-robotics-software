package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VisionClass" defined in "VisionClass_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VisionClass_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VisionClass_.idl instead.
*
*/
public class VisionClassPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.VisionClass>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::VisionClass_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e98ad8b22a9cfe744b731de5f59701486d7e30f255ebd64b0b6354bea78ff8d6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.VisionClass data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.VisionClass data) throws java.io.IOException
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.VisionClass data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.VisionClass data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getClassName().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.VisionClass data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_3(data.getClassId());

      if(data.getClassName().length() <= 255)
      cdr.write_type_d(data.getClassName());else
          throw new RuntimeException("class_name field exceeds the maximum length: %d > %d".formatted(data.getClassName().length(), 255));

   }

   public static void read(vision_msgs.msg.dds.VisionClass data, us.ihmc.idl.CDR cdr)
   {
      data.setClassId(cdr.read_type_3());
      	
      cdr.read_type_d(data.getClassName());	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.VisionClass data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_3("class_id", data.getClassId());
      ser.write_type_d("class_name", data.getClassName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.VisionClass data)
   {
      data.setClassId(ser.read_type_3("class_id"));
      ser.read_type_d("class_name", data.getClassName());
   }

   public static void staticCopy(vision_msgs.msg.dds.VisionClass src, vision_msgs.msg.dds.VisionClass dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.VisionClass createData()
   {
      return new vision_msgs.msg.dds.VisionClass();
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
   
   public void serialize(vision_msgs.msg.dds.VisionClass data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.VisionClass data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.VisionClass src, vision_msgs.msg.dds.VisionClass dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VisionClassPubSubType newInstance()
   {
      return new VisionClassPubSubType();
   }
}

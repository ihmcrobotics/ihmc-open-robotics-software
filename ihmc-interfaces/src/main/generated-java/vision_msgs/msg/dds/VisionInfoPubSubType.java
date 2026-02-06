package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VisionInfo" defined in "VisionInfo_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VisionInfo_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VisionInfo_.idl instead.
*
*/
public class VisionInfoPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.VisionInfo>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::VisionInfo_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a23ad4cc96eb078fc4dbdedd098c94fc474b832a0a3ce3d3569343852c73054d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.VisionInfo data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.VisionInfo data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.VisionInfo data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.VisionInfo data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getMethod().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getDatabaseLocation().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.VisionInfo data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getMethod().length() <= 255)
      cdr.write_type_d(data.getMethod());else
          throw new RuntimeException("method field exceeds the maximum length: %d > %d".formatted(data.getMethod().length(), 255));

      if(data.getDatabaseLocation().length() <= 255)
      cdr.write_type_d(data.getDatabaseLocation());else
          throw new RuntimeException("database_location field exceeds the maximum length: %d > %d".formatted(data.getDatabaseLocation().length(), 255));

      cdr.write_type_2(data.getDatabaseVersion());

   }

   public static void read(vision_msgs.msg.dds.VisionInfo data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_d(data.getMethod());	
      cdr.read_type_d(data.getDatabaseLocation());	
      data.setDatabaseVersion(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.VisionInfo data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_d("method", data.getMethod());
      ser.write_type_d("database_location", data.getDatabaseLocation());
      ser.write_type_2("database_version", data.getDatabaseVersion());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.VisionInfo data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_d("method", data.getMethod());
      ser.read_type_d("database_location", data.getDatabaseLocation());
      data.setDatabaseVersion(ser.read_type_2("database_version"));
   }

   public static void staticCopy(vision_msgs.msg.dds.VisionInfo src, vision_msgs.msg.dds.VisionInfo dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.VisionInfo createData()
   {
      return new vision_msgs.msg.dds.VisionInfo();
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
   
   public void serialize(vision_msgs.msg.dds.VisionInfo data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.VisionInfo data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.VisionInfo src, vision_msgs.msg.dds.VisionInfo dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VisionInfoPubSubType newInstance()
   {
      return new VisionInfoPubSubType();
   }
}

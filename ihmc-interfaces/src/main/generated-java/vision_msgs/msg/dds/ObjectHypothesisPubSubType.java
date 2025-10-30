package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ObjectHypothesis" defined in "ObjectHypothesis_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ObjectHypothesis_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ObjectHypothesis_.idl instead.
*
*/
public class ObjectHypothesisPubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.ObjectHypothesis>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::ObjectHypothesis_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fb5987211f847b59942b45f8ce2be4f8c6b7d4ef2aae391fecde9be78821ad0c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.ObjectHypothesis data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.ObjectHypothesis data) throws java.io.IOException
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
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.ObjectHypothesis data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.ObjectHypothesis data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getClassId().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.ObjectHypothesis data, us.ihmc.idl.CDR cdr)
   {
      if(data.getClassId().length() <= 255)
      cdr.write_type_d(data.getClassId());else
          throw new RuntimeException("class_id field exceeds the maximum length: %d > %d".formatted(data.getClassId().length(), 255));

      cdr.write_type_6(data.getScore());

   }

   public static void read(vision_msgs.msg.dds.ObjectHypothesis data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getClassId());	
      data.setScore(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.ObjectHypothesis data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("class_id", data.getClassId());
      ser.write_type_6("score", data.getScore());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.ObjectHypothesis data)
   {
      ser.read_type_d("class_id", data.getClassId());
      data.setScore(ser.read_type_6("score"));
   }

   public static void staticCopy(vision_msgs.msg.dds.ObjectHypothesis src, vision_msgs.msg.dds.ObjectHypothesis dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.ObjectHypothesis createData()
   {
      return new vision_msgs.msg.dds.ObjectHypothesis();
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
   
   public void serialize(vision_msgs.msg.dds.ObjectHypothesis data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.ObjectHypothesis data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.ObjectHypothesis src, vision_msgs.msg.dds.ObjectHypothesis dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ObjectHypothesisPubSubType newInstance()
   {
      return new ObjectHypothesisPubSubType();
   }
}

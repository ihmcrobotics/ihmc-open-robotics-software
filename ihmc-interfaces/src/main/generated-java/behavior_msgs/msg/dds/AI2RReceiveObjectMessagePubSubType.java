package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RReceiveObjectMessage" defined in "AI2RReceiveObjectMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RReceiveObjectMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RReceiveObjectMessage_.idl instead.
*
*/
public class AI2RReceiveObjectMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RReceiveObjectMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RReceiveObjectMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "818d23afba64ca96b910e0e95e1c8816841955e7788759ca487006c11ef1dc17";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RReceiveObjectMessage data) throws java.io.IOException
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
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getObjectName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getObjectName().length() <= 255)
      cdr.write_type_d(data.getObjectName());else
          throw new RuntimeException("object_name field exceeds the maximum length: %d > %d".formatted(data.getObjectName().length(), 255));

      cdr.write_type_9(data.getSide());

      cdr.write_type_6(data.getDistanceToObject());

   }

   public static void read(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getObjectName());	
      data.setSide(cdr.read_type_9());
      	
      data.setDistanceToObject(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("object_name", data.getObjectName());
      ser.write_type_9("side", data.getSide());
      ser.write_type_6("distance_to_object", data.getDistanceToObject());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RReceiveObjectMessage data)
   {
      ser.read_type_d("object_name", data.getObjectName());
      data.setSide(ser.read_type_9("side"));
      data.setDistanceToObject(ser.read_type_6("distance_to_object"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RReceiveObjectMessage src, behavior_msgs.msg.dds.AI2RReceiveObjectMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RReceiveObjectMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RReceiveObjectMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RReceiveObjectMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RReceiveObjectMessage src, behavior_msgs.msg.dds.AI2RReceiveObjectMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RReceiveObjectMessagePubSubType newInstance()
   {
      return new AI2RReceiveObjectMessagePubSubType();
   }
}

package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RNavigationMessage" defined in "AI2RNavigationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RNavigationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RNavigationMessage_.idl instead.
*
*/
public class AI2RNavigationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RNavigationMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RNavigationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2dedcb0be4210c321a0e50e4407b9a14d04b78fb03c733b05f7a72326d24aa56";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RNavigationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RNavigationMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNavigationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RNavigationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getSecondaryReferenceFrameName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getReferenceFrameName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RNavigationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getSpatialRelation());

      if(data.getSecondaryReferenceFrameName().length() <= 255)
      cdr.write_type_d(data.getSecondaryReferenceFrameName());else
          throw new RuntimeException("secondary_reference_frame_name field exceeds the maximum length: %d > %d".formatted(data.getSecondaryReferenceFrameName().length(), 255));

      if(data.getReferenceFrameName().length() <= 255)
      cdr.write_type_d(data.getReferenceFrameName());else
          throw new RuntimeException("reference_frame_name field exceeds the maximum length: %d > %d".formatted(data.getReferenceFrameName().length(), 255));

      cdr.write_type_6(data.getDistanceToFrame());

   }

   public static void read(behavior_msgs.msg.dds.AI2RNavigationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSpatialRelation(cdr.read_type_9());
      	
      cdr.read_type_d(data.getSecondaryReferenceFrameName());	
      cdr.read_type_d(data.getReferenceFrameName());	
      data.setDistanceToFrame(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RNavigationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("spatial_relation", data.getSpatialRelation());
      ser.write_type_d("secondary_reference_frame_name", data.getSecondaryReferenceFrameName());
      ser.write_type_d("reference_frame_name", data.getReferenceFrameName());
      ser.write_type_6("distance_to_frame", data.getDistanceToFrame());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RNavigationMessage data)
   {
      data.setSpatialRelation(ser.read_type_9("spatial_relation"));
      ser.read_type_d("secondary_reference_frame_name", data.getSecondaryReferenceFrameName());
      ser.read_type_d("reference_frame_name", data.getReferenceFrameName());
      data.setDistanceToFrame(ser.read_type_6("distance_to_frame"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RNavigationMessage src, behavior_msgs.msg.dds.AI2RNavigationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RNavigationMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RNavigationMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RNavigationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RNavigationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RNavigationMessage src, behavior_msgs.msg.dds.AI2RNavigationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RNavigationMessagePubSubType newInstance()
   {
      return new AI2RNavigationMessagePubSubType();
   }
}

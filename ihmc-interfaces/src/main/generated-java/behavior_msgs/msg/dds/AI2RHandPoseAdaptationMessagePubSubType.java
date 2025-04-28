package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RHandPoseAdaptationMessage" defined in "AI2RHandPoseAdaptationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RHandPoseAdaptationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RHandPoseAdaptationMessage_.idl instead.
*
*/
public class AI2RHandPoseAdaptationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RHandPoseAdaptationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8458b4da974760f87fa528adbf15d4f1dc68968f51facaa8f4ee86481ed23d65";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data) throws java.io.IOException
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
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getActionName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getReferenceFrameName().length() + 1;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getNewPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getNewOrientation(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getActionName().length() <= 255)
      cdr.write_type_d(data.getActionName());else
          throw new RuntimeException("action_name field exceeds the maximum length: %d > %d".formatted(data.getActionName().length(), 255));

      if(data.getReferenceFrameName().length() <= 255)
      cdr.write_type_d(data.getReferenceFrameName());else
          throw new RuntimeException("reference_frame_name field exceeds the maximum length: %d > %d".formatted(data.getReferenceFrameName().length(), 255));

      geometry_msgs.msg.dds.PointPubSubType.write(data.getNewPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getNewOrientation(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getActionName());	
      cdr.read_type_d(data.getReferenceFrameName());	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getNewPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getNewOrientation(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("action_name", data.getActionName());
      ser.write_type_d("reference_frame_name", data.getReferenceFrameName());
      ser.write_type_a("new_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getNewPosition());

      ser.write_type_a("new_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getNewOrientation());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data)
   {
      ser.read_type_d("action_name", data.getActionName());
      ser.read_type_d("reference_frame_name", data.getReferenceFrameName());
      ser.read_type_a("new_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getNewPosition());

      ser.read_type_a("new_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getNewOrientation());

   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage src, behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage src, behavior_msgs.msg.dds.AI2RHandPoseAdaptationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RHandPoseAdaptationMessagePubSubType newInstance()
   {
      return new AI2RHandPoseAdaptationMessagePubSubType();
   }
}

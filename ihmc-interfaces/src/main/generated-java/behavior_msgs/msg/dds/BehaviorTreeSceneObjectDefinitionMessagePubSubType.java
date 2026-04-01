package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BehaviorTreeSceneObjectDefinitionMessage" defined in "BehaviorTreeSceneObjectDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BehaviorTreeSceneObjectDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BehaviorTreeSceneObjectDefinitionMessage_.idl instead.
*
*/
public class BehaviorTreeSceneObjectDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::BehaviorTreeSceneObjectDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "85ead7d8bc80118edae9a9833f313394e04eebeed1c79f71c7e4939d055a6901";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data) throws java.io.IOException
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
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getYoloModelName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getYoloClassName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getCompositeFrameName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getCompositeFrameA().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getCompositeFrameB().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getObjectType());

      if(data.getYoloModelName().length() <= 255)
      cdr.write_type_d(data.getYoloModelName());else
          throw new RuntimeException("yolo_model_name field exceeds the maximum length: %d > %d".formatted(data.getYoloModelName().length(), 255));

      if(data.getYoloClassName().length() <= 255)
      cdr.write_type_d(data.getYoloClassName());else
          throw new RuntimeException("yolo_class_name field exceeds the maximum length: %d > %d".formatted(data.getYoloClassName().length(), 255));

      cdr.write_type_9(data.getFoundationPoseObjectType());

      cdr.write_type_3(data.getMinPostPoints());

      cdr.write_type_3(data.getMinRecessPoints());

      if(data.getCompositeFrameName().length() <= 255)
      cdr.write_type_d(data.getCompositeFrameName());else
          throw new RuntimeException("composite_frame_name field exceeds the maximum length: %d > %d".formatted(data.getCompositeFrameName().length(), 255));

      if(data.getCompositeFrameA().length() <= 255)
      cdr.write_type_d(data.getCompositeFrameA());else
          throw new RuntimeException("composite_frame_a field exceeds the maximum length: %d > %d".formatted(data.getCompositeFrameA().length(), 255));

      if(data.getCompositeFrameB().length() <= 255)
      cdr.write_type_d(data.getCompositeFrameB());else
          throw new RuntimeException("composite_frame_b field exceeds the maximum length: %d > %d".formatted(data.getCompositeFrameB().length(), 255));

      cdr.write_type_9(data.getCompositeFrameType());

      cdr.write_type_5(data.getCompositeDistance());

   }

   public static void read(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setObjectType(cdr.read_type_9());
      	
      cdr.read_type_d(data.getYoloModelName());	
      cdr.read_type_d(data.getYoloClassName());	
      data.setFoundationPoseObjectType(cdr.read_type_9());
      	
      data.setMinPostPoints(cdr.read_type_3());
      	
      data.setMinRecessPoints(cdr.read_type_3());
      	
      cdr.read_type_d(data.getCompositeFrameName());	
      cdr.read_type_d(data.getCompositeFrameA());	
      cdr.read_type_d(data.getCompositeFrameB());	
      data.setCompositeFrameType(cdr.read_type_9());
      	
      data.setCompositeDistance(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("object_type", data.getObjectType());
      ser.write_type_d("yolo_model_name", data.getYoloModelName());
      ser.write_type_d("yolo_class_name", data.getYoloClassName());
      ser.write_type_9("foundation_pose_object_type", data.getFoundationPoseObjectType());
      ser.write_type_3("min_post_points", data.getMinPostPoints());
      ser.write_type_3("min_recess_points", data.getMinRecessPoints());
      ser.write_type_d("composite_frame_name", data.getCompositeFrameName());
      ser.write_type_d("composite_frame_a", data.getCompositeFrameA());
      ser.write_type_d("composite_frame_b", data.getCompositeFrameB());
      ser.write_type_9("composite_frame_type", data.getCompositeFrameType());
      ser.write_type_5("composite_distance", data.getCompositeDistance());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data)
   {
      data.setObjectType(ser.read_type_9("object_type"));
      ser.read_type_d("yolo_model_name", data.getYoloModelName());
      ser.read_type_d("yolo_class_name", data.getYoloClassName());
      data.setFoundationPoseObjectType(ser.read_type_9("foundation_pose_object_type"));
      data.setMinPostPoints(ser.read_type_3("min_post_points"));
      data.setMinRecessPoints(ser.read_type_3("min_recess_points"));
      ser.read_type_d("composite_frame_name", data.getCompositeFrameName());
      ser.read_type_d("composite_frame_a", data.getCompositeFrameA());
      ser.read_type_d("composite_frame_b", data.getCompositeFrameB());
      data.setCompositeFrameType(ser.read_type_9("composite_frame_type"));
      data.setCompositeDistance(ser.read_type_5("composite_distance"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage src, behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage src, behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BehaviorTreeSceneObjectDefinitionMessagePubSubType newInstance()
   {
      return new BehaviorTreeSceneObjectDefinitionMessagePubSubType();
   }
}

package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectableSceneNodeMessage" defined in "DetectableSceneNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectableSceneNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectableSceneNodeMessage_.idl instead.
*
*/
public class DetectableSceneNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectableSceneNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectableSceneNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0ee2c191291ab8f338147dc1153067ec32bd853da7d89f1dff891a6da049dc3f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectableSceneNodeMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getTransformToWorld(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getArucoMarkerTransformToWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getName().length() <= 255)
      cdr.write_type_d(data.getName());else
          throw new RuntimeException("name field exceeds the maximum length");

      cdr.write_type_7(data.getCurrentlyDetected());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getTransformToWorld(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getArucoMarkerTransformToWorld(), cdr);
      cdr.write_type_7(data.getTrackDetectedPose());

      cdr.write_type_5(data.getBreakFrequency());

      cdr.write_type_5(data.getDistanceToDisableTracking());

      cdr.write_type_5(data.getCurrentDistanceToRobot());

   }

   public static void read(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getName());	
      data.setCurrentlyDetected(cdr.read_type_7());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getTransformToWorld(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getArucoMarkerTransformToWorld(), cdr);	
      data.setTrackDetectedPose(cdr.read_type_7());
      	
      data.setBreakFrequency(cdr.read_type_5());
      	
      data.setDistanceToDisableTracking(cdr.read_type_5());
      	
      data.setCurrentDistanceToRobot(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("name", data.getName());
      ser.write_type_7("currently_detected", data.getCurrentlyDetected());
      ser.write_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      ser.write_type_a("aruco_marker_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getArucoMarkerTransformToWorld());

      ser.write_type_7("track_detected_pose", data.getTrackDetectedPose());
      ser.write_type_5("break_frequency", data.getBreakFrequency());
      ser.write_type_5("distance_to_disable_tracking", data.getDistanceToDisableTracking());
      ser.write_type_5("current_distance_to_robot", data.getCurrentDistanceToRobot());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectableSceneNodeMessage data)
   {
      ser.read_type_d("name", data.getName());
      data.setCurrentlyDetected(ser.read_type_7("currently_detected"));
      ser.read_type_a("transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getTransformToWorld());

      ser.read_type_a("aruco_marker_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getArucoMarkerTransformToWorld());

      data.setTrackDetectedPose(ser.read_type_7("track_detected_pose"));
      data.setBreakFrequency(ser.read_type_5("break_frequency"));
      data.setDistanceToDisableTracking(ser.read_type_5("distance_to_disable_tracking"));
      data.setCurrentDistanceToRobot(ser.read_type_5("current_distance_to_robot"));
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectableSceneNodeMessage src, perception_msgs.msg.dds.DetectableSceneNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectableSceneNodeMessage createData()
   {
      return new perception_msgs.msg.dds.DetectableSceneNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectableSceneNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectableSceneNodeMessage src, perception_msgs.msg.dds.DetectableSceneNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectableSceneNodeMessagePubSubType newInstance()
   {
      return new DetectableSceneNodeMessagePubSubType();
   }
}

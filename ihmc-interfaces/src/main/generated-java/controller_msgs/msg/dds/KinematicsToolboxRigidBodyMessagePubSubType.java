package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxRigidBodyMessage" defined in "KinematicsToolboxRigidBodyMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxRigidBodyMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxRigidBodyMessage_.idl instead.
*
*/
public class KinematicsToolboxRigidBodyMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsToolboxRigidBodyMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredPositionInWorld(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredOrientationInWorld(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getControlFramePositionInEndEffector(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getControlFrameOrientationInEndEffector(), current_alignment);


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getAngularSelectionMatrix(), current_alignment);


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getLinearSelectionMatrix(), current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getAngularWeightMatrix(), current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getLinearWeightMatrix(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_2(data.getEndEffectorHashCode());


      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredPositionInWorld(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredOrientationInWorld(), cdr);

      geometry_msgs.msg.dds.PointPubSubType.write(data.getControlFramePositionInEndEffector(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getControlFrameOrientationInEndEffector(), cdr);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getAngularSelectionMatrix(), cdr);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getLinearSelectionMatrix(), cdr);

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getAngularWeightMatrix(), cdr);

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getLinearWeightMatrix(), cdr);
   }

   public static void read(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setEndEffectorHashCode(cdr.read_type_2());
      	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredPositionInWorld(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredOrientationInWorld(), cdr);	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getControlFramePositionInEndEffector(), cdr);	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getControlFrameOrientationInEndEffector(), cdr);	

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getAngularSelectionMatrix(), cdr);	

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getLinearSelectionMatrix(), cdr);	

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getAngularWeightMatrix(), cdr);	

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getLinearWeightMatrix(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_2("end_effector_hash_code", data.getEndEffectorHashCode());

      ser.write_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());


      ser.write_type_a("desired_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredOrientationInWorld());


      ser.write_type_a("control_frame_position_in_end_effector", new geometry_msgs.msg.dds.PointPubSubType(), data.getControlFramePositionInEndEffector());


      ser.write_type_a("control_frame_orientation_in_end_effector", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getControlFrameOrientationInEndEffector());


      ser.write_type_a("angular_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());


      ser.write_type_a("linear_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());


      ser.write_type_a("angular_weight_matrix", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getAngularWeightMatrix());


      ser.write_type_a("linear_weight_matrix", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getLinearWeightMatrix());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setEndEffectorHashCode(ser.read_type_2("end_effector_hash_code"));

      ser.read_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());


      ser.read_type_a("desired_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredOrientationInWorld());


      ser.read_type_a("control_frame_position_in_end_effector", new geometry_msgs.msg.dds.PointPubSubType(), data.getControlFramePositionInEndEffector());


      ser.read_type_a("control_frame_orientation_in_end_effector", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getControlFrameOrientationInEndEffector());


      ser.read_type_a("angular_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());


      ser.read_type_a("linear_selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());


      ser.read_type_a("angular_weight_matrix", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getAngularWeightMatrix());


      ser.read_type_a("linear_weight_matrix", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getLinearWeightMatrix());

   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage src, controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage src, controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxRigidBodyMessagePubSubType newInstance()
   {
      return new KinematicsToolboxRigidBodyMessagePubSubType();
   }
}

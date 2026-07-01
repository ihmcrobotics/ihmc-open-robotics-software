package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RobotPose" defined in "RobotPose_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RobotPose_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RobotPose_.idl instead.
*
*/
public class RobotPosePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.RobotPose>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::RobotPose_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "aecb74c69804c875471db71e133979cc7cd166217f4d1973e3ff26487753109a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.RobotPose data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.RobotPose data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RobotPose data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.RobotPose data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getJointNames().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getJointNames().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointPositions().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.RobotPose data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceNumber());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      if(data.getJointNames().size() <= 100)
      cdr.write_type_e(data.getJointNames());else
          throw new RuntimeException("joint_names field exceeds the maximum length: %d > %d".formatted(data.getJointNames().size(), 100));

      if(data.getJointPositions().size() <= 100)
      cdr.write_type_e(data.getJointPositions());else
          throw new RuntimeException("joint_positions field exceeds the maximum length: %d > %d".formatted(data.getJointPositions().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.RobotPose data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceNumber(cdr.read_type_12());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      cdr.read_type_e(data.getJointNames());	
      cdr.read_type_e(data.getJointPositions());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.RobotPose data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_number", data.getSequenceNumber());
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_e("joint_names", data.getJointNames());
      ser.write_type_e("joint_positions", data.getJointPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.RobotPose data)
   {
      data.setSequenceNumber(ser.read_type_12("sequence_number"));
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.read_type_e("joint_names", data.getJointNames());
      ser.read_type_e("joint_positions", data.getJointPositions());
   }

   public static void staticCopy(perception_msgs.msg.dds.RobotPose src, perception_msgs.msg.dds.RobotPose dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.RobotPose createData()
   {
      return new perception_msgs.msg.dds.RobotPose();
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
   
   public void serialize(perception_msgs.msg.dds.RobotPose data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.RobotPose data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.RobotPose src, perception_msgs.msg.dds.RobotPose dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotPosePubSubType newInstance()
   {
      return new RobotPosePubSubType();
   }
}

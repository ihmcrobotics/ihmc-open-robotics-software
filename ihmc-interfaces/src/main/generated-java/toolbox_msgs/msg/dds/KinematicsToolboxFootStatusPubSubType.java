package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxFootStatus" defined in "KinematicsToolboxFootStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxFootStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxFootStatus_.idl instead.
*
*/
public class KinematicsToolboxFootStatusPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxFootStatus>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxFootStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "44320bff25dc2dc23084afcf3448703fdbdffbb6ef6014fc8b9dc69537a79f82";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRelativeFootPositionFromPelvis(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRelativeFootOrientationFromPelvis(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRelativeFootPositionFromPelvisStepStart(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRelativeFootOrientationFromPelvisStepStart(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getSide());

      cdr.write_type_7(data.getFootInContact());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getRelativeFootPositionFromPelvis(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRelativeFootOrientationFromPelvis(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getRelativeFootPositionFromPelvisStepStart(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRelativeFootOrientationFromPelvisStepStart(), cdr);
   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSide(cdr.read_type_9());
      	
      data.setFootInContact(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRelativeFootPositionFromPelvis(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRelativeFootOrientationFromPelvis(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getRelativeFootPositionFromPelvisStepStart(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRelativeFootOrientationFromPelvisStepStart(), cdr);	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("side", data.getSide());
      ser.write_type_7("foot_in_contact", data.getFootInContact());
      ser.write_type_a("relative_foot_position_from_pelvis", new geometry_msgs.msg.dds.PointPubSubType(), data.getRelativeFootPositionFromPelvis());

      ser.write_type_a("relative_foot_orientation_from_pelvis", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRelativeFootOrientationFromPelvis());

      ser.write_type_a("relative_foot_position_from_pelvis_step_start", new geometry_msgs.msg.dds.PointPubSubType(), data.getRelativeFootPositionFromPelvisStepStart());

      ser.write_type_a("relative_foot_orientation_from_pelvis_step_start", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRelativeFootOrientationFromPelvisStepStart());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data)
   {
      data.setSide(ser.read_type_9("side"));
      data.setFootInContact(ser.read_type_7("foot_in_contact"));
      ser.read_type_a("relative_foot_position_from_pelvis", new geometry_msgs.msg.dds.PointPubSubType(), data.getRelativeFootPositionFromPelvis());

      ser.read_type_a("relative_foot_orientation_from_pelvis", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRelativeFootOrientationFromPelvis());

      ser.read_type_a("relative_foot_position_from_pelvis_step_start", new geometry_msgs.msg.dds.PointPubSubType(), data.getRelativeFootPositionFromPelvisStepStart());

      ser.read_type_a("relative_foot_orientation_from_pelvis_step_start", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRelativeFootOrientationFromPelvisStepStart());

   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus src, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxFootStatus createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxFootStatus();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxFootStatus src, toolbox_msgs.msg.dds.KinematicsToolboxFootStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxFootStatusPubSubType newInstance()
   {
      return new KinematicsToolboxFootStatusPubSubType();
   }
}

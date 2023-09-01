package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ReachingManifoldMessage" defined in "ReachingManifoldMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ReachingManifoldMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ReachingManifoldMessage_.idl instead.
*
*/
public class ReachingManifoldMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.ReachingManifoldMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::ReachingManifoldMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4969da9998fca7057eb8cbc32f4e3b189fd36b0c44068ea4b65681ac4bf82e55";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.ReachingManifoldMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.ReachingManifoldMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ReachingManifoldMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ReachingManifoldMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getManifoldOriginPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getManifoldOriginOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getManifoldConfigurationSpaceNames().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getManifoldLowerLimits().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getManifoldUpperLimits().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.ReachingManifoldMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getEndEffectorHashCode());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getManifoldOriginPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getManifoldOriginOrientation(), cdr);
      if(data.getManifoldConfigurationSpaceNames().size() <= 100)
      cdr.write_type_e(data.getManifoldConfigurationSpaceNames());else
          throw new RuntimeException("manifold_configuration_space_names field exceeds the maximum length");

      if(data.getManifoldLowerLimits().size() <= 100)
      cdr.write_type_e(data.getManifoldLowerLimits());else
          throw new RuntimeException("manifold_lower_limits field exceeds the maximum length");

      if(data.getManifoldUpperLimits().size() <= 100)
      cdr.write_type_e(data.getManifoldUpperLimits());else
          throw new RuntimeException("manifold_upper_limits field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.ReachingManifoldMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEndEffectorHashCode(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getManifoldOriginPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getManifoldOriginOrientation(), cdr);	
      cdr.read_type_e(data.getManifoldConfigurationSpaceNames());	
      cdr.read_type_e(data.getManifoldLowerLimits());	
      cdr.read_type_e(data.getManifoldUpperLimits());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.ReachingManifoldMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("end_effector_hash_code", data.getEndEffectorHashCode());
      ser.write_type_a("manifold_origin_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getManifoldOriginPosition());

      ser.write_type_a("manifold_origin_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getManifoldOriginOrientation());

      ser.write_type_e("manifold_configuration_space_names", data.getManifoldConfigurationSpaceNames());
      ser.write_type_e("manifold_lower_limits", data.getManifoldLowerLimits());
      ser.write_type_e("manifold_upper_limits", data.getManifoldUpperLimits());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.ReachingManifoldMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEndEffectorHashCode(ser.read_type_2("end_effector_hash_code"));
      ser.read_type_a("manifold_origin_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getManifoldOriginPosition());

      ser.read_type_a("manifold_origin_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getManifoldOriginOrientation());

      ser.read_type_e("manifold_configuration_space_names", data.getManifoldConfigurationSpaceNames());
      ser.read_type_e("manifold_lower_limits", data.getManifoldLowerLimits());
      ser.read_type_e("manifold_upper_limits", data.getManifoldUpperLimits());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.ReachingManifoldMessage src, toolbox_msgs.msg.dds.ReachingManifoldMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.ReachingManifoldMessage createData()
   {
      return new toolbox_msgs.msg.dds.ReachingManifoldMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.ReachingManifoldMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.ReachingManifoldMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.ReachingManifoldMessage src, toolbox_msgs.msg.dds.ReachingManifoldMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ReachingManifoldMessagePubSubType newInstance()
   {
      return new ReachingManifoldMessagePubSubType();
   }
}

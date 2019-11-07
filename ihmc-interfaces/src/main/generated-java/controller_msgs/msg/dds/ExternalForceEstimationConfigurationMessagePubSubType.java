package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ExternalForceEstimationConfigurationMessage" defined in "ExternalForceEstimationConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ExternalForceEstimationConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ExternalForceEstimationConfigurationMessage_.idl instead.
*
*/
public class ExternalForceEstimationConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ExternalForceEstimationConfigurationMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getExternalForcePosition(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getEstimatorGain());

      cdr.write_type_6(data.getSolverAlpha());

      cdr.write_type_2(data.getEndEffectorHashCode());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getExternalForcePosition(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEstimatorGain(cdr.read_type_6());
      	
      data.setSolverAlpha(cdr.read_type_6());
      	
      data.setEndEffectorHashCode(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getExternalForcePosition(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("estimator_gain", data.getEstimatorGain());
      ser.write_type_6("solver_alpha", data.getSolverAlpha());
      ser.write_type_2("end_effector_hash_code", data.getEndEffectorHashCode());
      ser.write_type_a("external_force_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getExternalForcePosition());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEstimatorGain(ser.read_type_6("estimator_gain"));
      data.setSolverAlpha(ser.read_type_6("solver_alpha"));
      data.setEndEffectorHashCode(ser.read_type_2("end_effector_hash_code"));
      ser.read_type_a("external_force_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getExternalForcePosition());

   }

   public static void staticCopy(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage src, controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage createData()
   {
      return new controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage src, controller_msgs.msg.dds.ExternalForceEstimationConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ExternalForceEstimationConfigurationMessagePubSubType newInstance()
   {
      return new ExternalForceEstimationConfigurationMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RigidBodyExplorationConfigurationMessage" defined in "RigidBodyExplorationConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RigidBodyExplorationConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RigidBodyExplorationConfigurationMessage_.idl instead.
*
*/
public class RigidBodyExplorationConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RigidBodyExplorationConfigurationMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getConfigurationSpaceNamesToExplore().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getExplorationRangeUpperLimits().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getExplorationRangeLowerLimits().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getRigidBodyNameBasedHashCode());

      if(data.getConfigurationSpaceNamesToExplore().size() <= 100)
      cdr.write_type_e(data.getConfigurationSpaceNamesToExplore());else
          throw new RuntimeException("configuration_space_names_to_explore field exceeds the maximum length");

      if(data.getExplorationRangeUpperLimits().size() <= 100)
      cdr.write_type_e(data.getExplorationRangeUpperLimits());else
          throw new RuntimeException("exploration_range_upper_limits field exceeds the maximum length");

      if(data.getExplorationRangeLowerLimits().size() <= 100)
      cdr.write_type_e(data.getExplorationRangeLowerLimits());else
          throw new RuntimeException("exploration_range_lower_limits field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRigidBodyNameBasedHashCode(cdr.read_type_11());
      	
      cdr.read_type_e(data.getConfigurationSpaceNamesToExplore());	
      cdr.read_type_e(data.getExplorationRangeUpperLimits());	
      cdr.read_type_e(data.getExplorationRangeLowerLimits());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("rigid_body_name_based_hash_code", data.getRigidBodyNameBasedHashCode());
      ser.write_type_e("configuration_space_names_to_explore", data.getConfigurationSpaceNamesToExplore());
      ser.write_type_e("exploration_range_upper_limits", data.getExplorationRangeUpperLimits());
      ser.write_type_e("exploration_range_lower_limits", data.getExplorationRangeLowerLimits());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRigidBodyNameBasedHashCode(ser.read_type_11("rigid_body_name_based_hash_code"));
      ser.read_type_e("configuration_space_names_to_explore", data.getConfigurationSpaceNamesToExplore());
      ser.read_type_e("exploration_range_upper_limits", data.getExplorationRangeUpperLimits());
      ser.read_type_e("exploration_range_lower_limits", data.getExplorationRangeLowerLimits());
   }

   public static void staticCopy(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage src, controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage createData()
   {
      return new controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage();
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
   
   public void serialize(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage src, controller_msgs.msg.dds.RigidBodyExplorationConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RigidBodyExplorationConfigurationMessagePubSubType newInstance()
   {
      return new RigidBodyExplorationConfigurationMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "KinematicsToolboxConfigurationMessage" defined in "KinematicsToolboxConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from KinematicsToolboxConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit KinematicsToolboxConfigurationMessage_.idl instead.
 */
public class KinematicsToolboxConfigurationMessagePubSubType
      implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsToolboxConfigurationMessage_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public KinematicsToolboxConfigurationMessagePubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPrivilegedRootJointPosition(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getPrivilegedRootJointOrientation(), current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getPrivilegedJointNameBasedHashCodes().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getPrivilegedJointAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPrivilegedRootJointPosition(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getPrivilegedRootJointOrientation(), cdr);

      if (data.getPrivilegedJointNameBasedHashCodes().size() <= 100)
         cdr.write_type_e(data.getPrivilegedJointNameBasedHashCodes());
      else
         throw new RuntimeException("privileged_joint_name_based_hash_codes field exceeds the maximum length");

      if (data.getPrivilegedJointAngles().size() <= 100)
         cdr.write_type_e(data.getPrivilegedJointAngles());
      else
         throw new RuntimeException("privileged_joint_angles field exceeds the maximum length");
   }

   public static void read(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.PointPubSubType.read(data.getPrivilegedRootJointPosition(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getPrivilegedRootJointOrientation(), cdr);

      cdr.read_type_e(data.getPrivilegedJointNameBasedHashCodes());

      cdr.read_type_e(data.getPrivilegedJointAngles());
   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage src,
                                 controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("privileged_root_joint_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPrivilegedRootJointPosition());

      ser.write_type_a("privileged_root_joint_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getPrivilegedRootJointOrientation());

      ser.write_type_e("privileged_joint_name_based_hash_codes", data.getPrivilegedJointNameBasedHashCodes());

      ser.write_type_e("privileged_joint_angles", data.getPrivilegedJointAngles());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data)
   {
      ser.read_type_a("privileged_root_joint_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPrivilegedRootJointPosition());

      ser.read_type_a("privileged_root_joint_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getPrivilegedRootJointOrientation());

      ser.read_type_e("privileged_joint_name_based_hash_codes", data.getPrivilegedJointNameBasedHashCodes());

      ser.read_type_e("privileged_joint_angles", data.getPrivilegedJointAngles());
   }

   @Override
   public controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage();
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

   public void serialize(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage src, controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxConfigurationMessagePubSubType newInstance()
   {
      return new KinematicsToolboxConfigurationMessagePubSubType();
   }
}

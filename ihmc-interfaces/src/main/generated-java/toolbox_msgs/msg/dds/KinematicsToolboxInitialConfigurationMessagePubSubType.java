package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxInitialConfigurationMessage" defined in "KinematicsToolboxInitialConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxInitialConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxInitialConfigurationMessage_.idl instead.
*
*/
public class KinematicsToolboxInitialConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxInitialConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5ecb7436690ed7cf4cc4d329dc1eeb8f4eba19e578c31d91496994fcea022898";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getInitialJointHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getInitialJointAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getInitialJointHashCodes().size() <= 100)
      cdr.write_type_e(data.getInitialJointHashCodes());else
          throw new RuntimeException("initial_joint_hash_codes field exceeds the maximum length");

      if(data.getInitialJointAngles().size() <= 100)
      cdr.write_type_e(data.getInitialJointAngles());else
          throw new RuntimeException("initial_joint_angles field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getInitialJointHashCodes());	
      cdr.read_type_e(data.getInitialJointAngles());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("initial_joint_hash_codes", data.getInitialJointHashCodes());
      ser.write_type_e("initial_joint_angles", data.getInitialJointAngles());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("initial_joint_hash_codes", data.getInitialJointHashCodes());
      ser.read_type_e("initial_joint_angles", data.getInitialJointAngles());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxInitialConfigurationMessagePubSubType newInstance()
   {
      return new KinematicsToolboxInitialConfigurationMessagePubSubType();
   }
}

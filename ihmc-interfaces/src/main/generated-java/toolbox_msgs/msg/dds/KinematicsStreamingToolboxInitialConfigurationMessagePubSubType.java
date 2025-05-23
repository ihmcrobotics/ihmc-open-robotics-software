package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsStreamingToolboxInitialConfigurationMessage" defined in "KinematicsStreamingToolboxInitialConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsStreamingToolboxInitialConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsStreamingToolboxInitialConfigurationMessage_.idl instead.
*
*/
public class KinematicsStreamingToolboxInitialConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsStreamingToolboxInitialConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "be90afa650d595cb3cdc7d64d11b00609c0e72d1adddce06a7632390d48659ff";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getInitialJointHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getInitialJointAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      if(data.getInitialJointHashCodes().size() <= 100)
      cdr.write_type_e(data.getInitialJointHashCodes());else
          throw new RuntimeException("initial_joint_hash_codes field exceeds the maximum length: %d > %d".formatted(data.getInitialJointHashCodes().size(), 100));

      if(data.getInitialJointAngles().size() <= 100)
      cdr.write_type_e(data.getInitialJointAngles());else
          throw new RuntimeException("initial_joint_angles field exceeds the maximum length: %d > %d".formatted(data.getInitialJointAngles().size(), 100));

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      cdr.read_type_e(data.getInitialJointHashCodes());	
      cdr.read_type_e(data.getInitialJointAngles());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_e("initial_joint_hash_codes", data.getInitialJointHashCodes());
      ser.write_type_e("initial_joint_angles", data.getInitialJointAngles());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      ser.read_type_e("initial_joint_hash_codes", data.getInitialJointHashCodes());
      ser.read_type_e("initial_joint_angles", data.getInitialJointAngles());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage src, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsStreamingToolboxInitialConfigurationMessagePubSubType newInstance()
   {
      return new KinematicsStreamingToolboxInitialConfigurationMessagePubSubType();
   }
}

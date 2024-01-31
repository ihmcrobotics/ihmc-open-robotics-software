package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxOneDoFJointMessage" defined in "KinematicsToolboxOneDoFJointMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxOneDoFJointMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxOneDoFJointMessage_.idl instead.
*
*/
public class KinematicsToolboxOneDoFJointMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxOneDoFJointMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "90ecace412c00eab0ec4667102d1928f97825b5fdbd3c8db701e7e66756fb2ee";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getJointHashCode());

      cdr.write_type_6(data.getDesiredPosition());

      cdr.write_type_6(data.getWeight());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setJointHashCode(cdr.read_type_2());
      	
      data.setDesiredPosition(cdr.read_type_6());
      	
      data.setWeight(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("joint_hash_code", data.getJointHashCode());
      ser.write_type_6("desired_position", data.getDesiredPosition());
      ser.write_type_6("weight", data.getWeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setJointHashCode(ser.read_type_2("joint_hash_code"));
      data.setDesiredPosition(ser.read_type_6("desired_position"));
      data.setWeight(ser.read_type_6("weight"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage src, toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage src, toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxOneDoFJointMessagePubSubType newInstance()
   {
      return new KinematicsToolboxOneDoFJointMessagePubSubType();
   }
}

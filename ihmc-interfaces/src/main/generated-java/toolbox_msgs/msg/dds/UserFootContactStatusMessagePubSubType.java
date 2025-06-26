package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "UserFootContactStatusMessage" defined in "UserFootContactStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from UserFootContactStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit UserFootContactStatusMessage_.idl instead.
*
*/
public class UserFootContactStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.UserFootContactStatusMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::UserFootContactStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f27f67cd1a32f06cf97f3cff0dd36e3404b0be28719cb3dfadde483a7c602c00";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.UserFootContactStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.UserFootContactStatusMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.UserFootContactStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.UserFootContactStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.UserFootContactStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getLeftFootInContact());

      cdr.write_type_7(data.getRightFootInContact());

   }

   public static void read(toolbox_msgs.msg.dds.UserFootContactStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setLeftFootInContact(cdr.read_type_7());
      	
      data.setRightFootInContact(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.UserFootContactStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("left_foot_in_contact", data.getLeftFootInContact());
      ser.write_type_7("right_foot_in_contact", data.getRightFootInContact());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.UserFootContactStatusMessage data)
   {
      data.setLeftFootInContact(ser.read_type_7("left_foot_in_contact"));
      data.setRightFootInContact(ser.read_type_7("right_foot_in_contact"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.UserFootContactStatusMessage src, toolbox_msgs.msg.dds.UserFootContactStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.UserFootContactStatusMessage createData()
   {
      return new toolbox_msgs.msg.dds.UserFootContactStatusMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.UserFootContactStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.UserFootContactStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.UserFootContactStatusMessage src, toolbox_msgs.msg.dds.UserFootContactStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public UserFootContactStatusMessagePubSubType newInstance()
   {
      return new UserFootContactStatusMessagePubSubType();
   }
}

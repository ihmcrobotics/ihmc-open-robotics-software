package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ControllerReleaseGoalMessage" defined in "ControllerReleaseGoalMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ControllerReleaseGoalMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ControllerReleaseGoalMessage_.idl instead.
*
*/
public class ControllerReleaseGoalMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ControllerReleaseGoalMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ControllerReleaseGoalMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "74a3cd10870be1a1d850da1cecfcf826507162f98d65169f8545040067747e71";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ControllerReleaseGoalMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ControllerReleaseGoalMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerReleaseGoalMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerReleaseGoalMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ControllerReleaseGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getReleaseGoal());

   }

   public static void read(controller_msgs.msg.dds.ControllerReleaseGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setReleaseGoal(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ControllerReleaseGoalMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("release_goal", data.getReleaseGoal());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ControllerReleaseGoalMessage data)
   {
      data.setReleaseGoal(ser.read_type_7("release_goal"));   }

   public static void staticCopy(controller_msgs.msg.dds.ControllerReleaseGoalMessage src, controller_msgs.msg.dds.ControllerReleaseGoalMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ControllerReleaseGoalMessage createData()
   {
      return new controller_msgs.msg.dds.ControllerReleaseGoalMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ControllerReleaseGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ControllerReleaseGoalMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ControllerReleaseGoalMessage src, controller_msgs.msg.dds.ControllerReleaseGoalMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ControllerReleaseGoalMessagePubSubType newInstance()
   {
      return new ControllerReleaseGoalMessagePubSubType();
   }
}

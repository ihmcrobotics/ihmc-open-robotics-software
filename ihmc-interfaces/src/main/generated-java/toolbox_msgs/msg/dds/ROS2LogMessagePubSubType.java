package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ROS2LogMessage" defined in "ROS2LogMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ROS2LogMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ROS2LogMessage_.idl instead.
*
*/
public class ROS2LogMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.ROS2LogMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::ROS2LogMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d7c52a1992eaab3574f9b0120c1cc0955b70717027a483e6445fd9c8d72d28fb";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.ROS2LogMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.ROS2LogMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ROS2LogMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.ROS2LogMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.ROS2LogMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRequestedState());

   }

   public static void read(toolbox_msgs.msg.dds.ROS2LogMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRequestedState(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.ROS2LogMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("requested_state", data.getRequestedState());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.ROS2LogMessage data)
   {
      data.setRequestedState(ser.read_type_9("requested_state"));   }

   public static void staticCopy(toolbox_msgs.msg.dds.ROS2LogMessage src, toolbox_msgs.msg.dds.ROS2LogMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.ROS2LogMessage createData()
   {
      return new toolbox_msgs.msg.dds.ROS2LogMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.ROS2LogMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.ROS2LogMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.ROS2LogMessage src, toolbox_msgs.msg.dds.ROS2LogMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ROS2LogMessagePubSubType newInstance()
   {
      return new ROS2LogMessagePubSubType();
   }
}

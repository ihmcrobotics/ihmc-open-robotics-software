package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkingControllerPreviewOutputMessage" defined in "WalkingControllerPreviewOutputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkingControllerPreviewOutputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkingControllerPreviewOutputMessage_.idl instead.
*
*/
public class WalkingControllerPreviewOutputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::WalkingControllerPreviewOutputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4c3f3e7c410605961980fc23b3c418faad79bef349094ddff1f55d6a38ff0d9d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 1000; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRobotConfigurations().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getCdrSerializedSize(data.getRobotConfigurations().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getFrameDt());

      if(data.getRobotConfigurations().size() <= 1000)
      cdr.write_type_e(data.getRobotConfigurations());else
          throw new RuntimeException("robot_configurations field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setFrameDt(cdr.read_type_6());
      	
      cdr.read_type_e(data.getRobotConfigurations());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("frame_dt", data.getFrameDt());
      ser.write_type_e("robot_configurations", data.getRobotConfigurations());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setFrameDt(ser.read_type_6("frame_dt"));
      ser.read_type_e("robot_configurations", data.getRobotConfigurations());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage src, toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage createData()
   {
      return new toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage src, toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkingControllerPreviewOutputMessagePubSubType newInstance()
   {
      return new WalkingControllerPreviewOutputMessagePubSubType();
   }
}

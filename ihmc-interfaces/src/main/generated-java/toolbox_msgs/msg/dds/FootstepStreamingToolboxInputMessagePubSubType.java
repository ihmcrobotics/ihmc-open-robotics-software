package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepStreamingToolboxInputMessage" defined in "FootstepStreamingToolboxInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepStreamingToolboxInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepStreamingToolboxInputMessage_.idl instead.
*
*/
public class FootstepStreamingToolboxInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepStreamingToolboxInputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b64df09a62e2608a16258335e10e976ad58c30924715bdae003fb86f13a09ddf";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 2; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSide().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.FootstepStreamingToolboxSideMessagePubSubType.getCdrSerializedSize(data.getSide().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_6(data.getRobotStepDuration());

      cdr.write_type_6(data.getRobotSwingDuration());

      cdr.write_type_6(data.getRobotStepElapsedTime());

      cdr.write_type_9(data.getRobotSwingSide());

      cdr.write_type_7(data.getIsRobotSwingFootLanding());

      if(data.getSide().size() <= 2)
      cdr.write_type_e(data.getSide());else
          throw new RuntimeException("side field exceeds the maximum length: %d > %d".formatted(data.getSide().size(), 2));

   }

   public static void read(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setRobotStepDuration(cdr.read_type_6());
      	
      data.setRobotSwingDuration(cdr.read_type_6());
      	
      data.setRobotStepElapsedTime(cdr.read_type_6());
      	
      data.setRobotSwingSide(cdr.read_type_9());
      	
      data.setIsRobotSwingFootLanding(cdr.read_type_7());
      	
      cdr.read_type_e(data.getSide());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_6("robot_step_duration", data.getRobotStepDuration());
      ser.write_type_6("robot_swing_duration", data.getRobotSwingDuration());
      ser.write_type_6("robot_step_elapsed_time", data.getRobotStepElapsedTime());
      ser.write_type_9("robot_swing_side", data.getRobotSwingSide());
      ser.write_type_7("is_robot_swing_foot_landing", data.getIsRobotSwingFootLanding());
      ser.write_type_e("side", data.getSide());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setRobotStepDuration(ser.read_type_6("robot_step_duration"));
      data.setRobotSwingDuration(ser.read_type_6("robot_swing_duration"));
      data.setRobotStepElapsedTime(ser.read_type_6("robot_step_elapsed_time"));
      data.setRobotSwingSide(ser.read_type_9("robot_swing_side"));
      data.setIsRobotSwingFootLanding(ser.read_type_7("is_robot_swing_foot_landing"));
      ser.read_type_e("side", data.getSide());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage src, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage src, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepStreamingToolboxInputMessagePubSubType newInstance()
   {
      return new FootstepStreamingToolboxInputMessagePubSubType();
   }
}

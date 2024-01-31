package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SwingPlanningRequestPacket" defined in "SwingPlanningRequestPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SwingPlanningRequestPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SwingPlanningRequestPacket_.idl instead.
*
*/
public class SwingPlanningRequestPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.SwingPlanningRequestPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::SwingPlanningRequestPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ac6f8510725a3f8c0e50772c9d3e56e6d576a60aed233b4b793232b44101bb14";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.SwingPlanningRequestPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRequestedSwingPlanner());

      cdr.write_type_7(data.getGenerateLog());

   }

   public static void read(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRequestedSwingPlanner(cdr.read_type_9());
      	
      data.setGenerateLog(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("requested_swing_planner", data.getRequestedSwingPlanner());
      ser.write_type_7("generate_log", data.getGenerateLog());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.SwingPlanningRequestPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRequestedSwingPlanner(ser.read_type_9("requested_swing_planner"));
      data.setGenerateLog(ser.read_type_7("generate_log"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.SwingPlanningRequestPacket src, toolbox_msgs.msg.dds.SwingPlanningRequestPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.SwingPlanningRequestPacket createData()
   {
      return new toolbox_msgs.msg.dds.SwingPlanningRequestPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.SwingPlanningRequestPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.SwingPlanningRequestPacket src, toolbox_msgs.msg.dds.SwingPlanningRequestPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SwingPlanningRequestPacketPubSubType newInstance()
   {
      return new SwingPlanningRequestPacketPubSubType();
   }
}

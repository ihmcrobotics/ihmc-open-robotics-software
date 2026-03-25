package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ControllerWaypointGoalListMessage" defined in "ControllerWaypointGoalListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ControllerWaypointGoalListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ControllerWaypointGoalListMessage_.idl instead.
*
*/
public class ControllerWaypointGoalListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ControllerWaypointGoalListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ControllerWaypointGoalListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "37b62c13a5a55c99b8057dfd3675ced12661566eb9a2219a4772b8fedfea253d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ControllerWaypointGoalListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.ControllerWaypointGoalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getWaypoints().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.ControllerWaypointGoalMessagePubSubType.getCdrSerializedSize(data.getWaypoints().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      if(data.getWaypoints().size() <= 50)
      cdr.write_type_e(data.getWaypoints());else
          throw new RuntimeException("waypoints field exceeds the maximum length: %d > %d".formatted(data.getWaypoints().size(), 50));

   }

   public static void read(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      cdr.read_type_e(data.getWaypoints());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_e("waypoints", data.getWaypoints());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ControllerWaypointGoalListMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      ser.read_type_e("waypoints", data.getWaypoints());
   }

   public static void staticCopy(controller_msgs.msg.dds.ControllerWaypointGoalListMessage src, controller_msgs.msg.dds.ControllerWaypointGoalListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ControllerWaypointGoalListMessage createData()
   {
      return new controller_msgs.msg.dds.ControllerWaypointGoalListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ControllerWaypointGoalListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ControllerWaypointGoalListMessage src, controller_msgs.msg.dds.ControllerWaypointGoalListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ControllerWaypointGoalListMessagePubSubType newInstance()
   {
      return new ControllerWaypointGoalListMessagePubSubType();
   }
}

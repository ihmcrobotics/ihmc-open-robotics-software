package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepNodeDataListMessage" defined in "FootstepNodeDataListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepNodeDataListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepNodeDataListMessage_.idl instead.
*
*/
public class FootstepNodeDataListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepNodeDataListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepNodeDataListMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepNodeDataListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepNodeDataListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200000; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepNodeDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepNodeDataListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepNodeDataListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getNodeData().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepNodeDataMessagePubSubType.getCdrSerializedSize(data.getNodeData().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepNodeDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getPlanId());

      if(data.getNodeData().size() <= 200000)
      cdr.write_type_e(data.getNodeData());else
          throw new RuntimeException("node_data field exceeds the maximum length");

      cdr.write_type_7(data.getIsFootstepGraph());

   }

   public static void read(controller_msgs.msg.dds.FootstepNodeDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setPlanId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getNodeData());	
      data.setIsFootstepGraph(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepNodeDataListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("plan_id", data.getPlanId());
      ser.write_type_e("node_data", data.getNodeData());
      ser.write_type_7("is_footstep_graph", data.getIsFootstepGraph());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepNodeDataListMessage data)
   {
      data.setPlanId(ser.read_type_4("plan_id"));
      ser.read_type_e("node_data", data.getNodeData());
      data.setIsFootstepGraph(ser.read_type_7("is_footstep_graph"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepNodeDataListMessage src, controller_msgs.msg.dds.FootstepNodeDataListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepNodeDataListMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepNodeDataListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepNodeDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepNodeDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepNodeDataListMessage src, controller_msgs.msg.dds.FootstepNodeDataListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepNodeDataListMessagePubSubType newInstance()
   {
      return new FootstepNodeDataListMessagePubSubType();
   }
}

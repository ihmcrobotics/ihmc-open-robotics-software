package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerLatticeMapMessage" defined in "FootstepPlannerLatticeMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerLatticeMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerLatticeMapMessage_.idl instead.
*
*/
public class FootstepPlannerLatticeMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlannerLatticeMapMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10000; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getLatticeNodes().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessagePubSubType.getCdrSerializedSize(data.getLatticeNodes().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getPlanId());

      if(data.getLatticeNodes().size() <= 10000)
      cdr.write_type_e(data.getLatticeNodes());else
          throw new RuntimeException("lattice_nodes field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setPlanId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getLatticeNodes());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("plan_id", data.getPlanId());
      ser.write_type_e("lattice_nodes", data.getLatticeNodes());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data)
   {
      data.setPlanId(ser.read_type_4("plan_id"));
      ser.read_type_e("lattice_nodes", data.getLatticeNodes());
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage src, controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage src, controller_msgs.msg.dds.FootstepPlannerLatticeMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerLatticeMapMessagePubSubType newInstance()
   {
      return new FootstepPlannerLatticeMapMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerLatticeNodeMessage" defined in "FootstepPlannerLatticeNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerLatticeNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerLatticeNodeMessage_.idl instead.
*
*/
public class FootstepPlannerLatticeNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlannerLatticeNodeMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getXIndex());

      cdr.write_type_2(data.getYIndex());

      cdr.write_type_2(data.getYawIndex());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_2(data.getNodeIndex());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setXIndex(cdr.read_type_2());
      	
      data.setYIndex(cdr.read_type_2());
      	
      data.setYawIndex(cdr.read_type_2());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setNodeIndex(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("x_index", data.getXIndex());
      ser.write_type_2("y_index", data.getYIndex());
      ser.write_type_2("yaw_index", data.getYawIndex());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_2("node_index", data.getNodeIndex());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data)
   {
      data.setXIndex(ser.read_type_2("x_index"));
      data.setYIndex(ser.read_type_2("y_index"));
      data.setYawIndex(ser.read_type_2("yaw_index"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setNodeIndex(ser.read_type_2("node_index"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage src, controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage src, controller_msgs.msg.dds.FootstepPlannerLatticeNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerLatticeNodeMessagePubSubType newInstance()
   {
      return new FootstepPlannerLatticeNodeMessagePubSubType();
   }
}

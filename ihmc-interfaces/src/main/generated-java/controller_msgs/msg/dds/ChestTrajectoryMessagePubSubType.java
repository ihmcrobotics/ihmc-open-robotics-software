package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ChestTrajectoryMessage" defined in "ChestTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ChestTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ChestTrajectoryMessage_.idl instead.
*
*/
public class ChestTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ChestTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ChestTrajectoryMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ChestTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ChestTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ChestTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ChestTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getSo3Trajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ChestTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.write(data.getSo3Trajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ChestTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.read(data.getSo3Trajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ChestTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("so3_trajectory", new controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ChestTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("so3_trajectory", new controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.ChestTrajectoryMessage src, controller_msgs.msg.dds.ChestTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ChestTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.ChestTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ChestTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ChestTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ChestTrajectoryMessage src, controller_msgs.msg.dds.ChestTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ChestTrajectoryMessagePubSubType newInstance()
   {
      return new ChestTrajectoryMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SE3PIDGainsTrajectoryMessage" defined in "SE3PIDGainsTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SE3PIDGainsTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SE3PIDGainsTrajectoryMessage_.idl instead.
*
*/
public class SE3PIDGainsTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SE3PIDGainsTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "18a9ffcf2c5a4f7ec8db4ffbd3db4c8888f187e6b1ecb820365201b8a108ec90";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPidGainsTrajectoryPoints().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getPidGainsTrajectoryPoints().get(i0), current_alignment);}

      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getPidGainsTrajectoryPoints().size() <= 200)
      cdr.write_type_e(data.getPidGainsTrajectoryPoints());else
          throw new RuntimeException("pid_gains_trajectory_points field exceeds the maximum length");

      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);
   }

   public static void read(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getPidGainsTrajectoryPoints());	
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("pid_gains_trajectory_points", data.getPidGainsTrajectoryPoints());
      ser.write_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("pid_gains_trajectory_points", data.getPidGainsTrajectoryPoints());
      ser.read_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   public static void staticCopy(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage src, controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage src, controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SE3PIDGainsTrajectoryMessagePubSubType newInstance()
   {
      return new SE3PIDGainsTrajectoryMessagePubSubType();
   }
}

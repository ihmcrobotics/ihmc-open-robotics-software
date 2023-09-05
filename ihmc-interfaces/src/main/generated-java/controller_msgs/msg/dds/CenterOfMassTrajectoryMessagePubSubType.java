package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CenterOfMassTrajectoryMessage" defined in "CenterOfMassTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CenterOfMassTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CenterOfMassTrajectoryMessage_.idl instead.
*
*/
public class CenterOfMassTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CenterOfMassTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CenterOfMassTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b2def8c631881a89794021f60e7a7d778f4fa26aba88aa8c85c971e7a018f3ab";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.getCdrSerializedSize(data.getEuclideanTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.write(data.getEuclideanTrajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.read(data.getEuclideanTrajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("euclidean_trajectory", new ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("euclidean_trajectory", new ihmc_common_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType(), data.getEuclideanTrajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage src, controller_msgs.msg.dds.CenterOfMassTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CenterOfMassTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.CenterOfMassTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CenterOfMassTrajectoryMessage src, controller_msgs.msg.dds.CenterOfMassTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CenterOfMassTrajectoryMessagePubSubType newInstance()
   {
      return new CenterOfMassTrajectoryMessagePubSubType();
   }
}

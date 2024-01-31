package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactTrajectoryStatus" defined in "MultiContactTrajectoryStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactTrajectoryStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactTrajectoryStatus_.idl instead.
*
*/
public class MultiContactTrajectoryStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.MultiContactTrajectoryStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::MultiContactTrajectoryStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0045d9a5c4df70f58cc3aec4c3c1840d90348a912033edd01effe4fd7d018854";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.MultiContactTrajectoryStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.MultiContactTrajectoryStatus data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactTrajectoryStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactTrajectoryStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.MultiContactTrajectoryStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getTrajectoryStatus());

   }

   public static void read(controller_msgs.msg.dds.MultiContactTrajectoryStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTrajectoryStatus(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.MultiContactTrajectoryStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("trajectory_status", data.getTrajectoryStatus());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.MultiContactTrajectoryStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTrajectoryStatus(ser.read_type_9("trajectory_status"));
   }

   public static void staticCopy(controller_msgs.msg.dds.MultiContactTrajectoryStatus src, controller_msgs.msg.dds.MultiContactTrajectoryStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.MultiContactTrajectoryStatus createData()
   {
      return new controller_msgs.msg.dds.MultiContactTrajectoryStatus();
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
   
   public void serialize(controller_msgs.msg.dds.MultiContactTrajectoryStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.MultiContactTrajectoryStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.MultiContactTrajectoryStatus src, controller_msgs.msg.dds.MultiContactTrajectoryStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactTrajectoryStatusPubSubType newInstance()
   {
      return new MultiContactTrajectoryStatusPubSubType();
   }
}

package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FrameInformation" defined in "FrameInformation_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FrameInformation_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FrameInformation_.idl instead.
*
*/
public class FrameInformationPubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.FrameInformation>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::FrameInformation_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e4e3d2374995103dabba60094d4189adca69a58f2d7710e7de27b1a6d90b4ced";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.FrameInformation data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.FrameInformation data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.FrameInformation data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.FrameInformation data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.FrameInformation data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTrajectoryReferenceFrameId());

      cdr.write_type_11(data.getDataReferenceFrameId());

   }

   public static void read(ihmc_common_msgs.msg.dds.FrameInformation data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTrajectoryReferenceFrameId(cdr.read_type_11());
      	
      data.setDataReferenceFrameId(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.FrameInformation data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("trajectory_reference_frame_id", data.getTrajectoryReferenceFrameId());
      ser.write_type_11("data_reference_frame_id", data.getDataReferenceFrameId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.FrameInformation data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTrajectoryReferenceFrameId(ser.read_type_11("trajectory_reference_frame_id"));
      data.setDataReferenceFrameId(ser.read_type_11("data_reference_frame_id"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.FrameInformation src, ihmc_common_msgs.msg.dds.FrameInformation dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.FrameInformation createData()
   {
      return new ihmc_common_msgs.msg.dds.FrameInformation();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.FrameInformation data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.FrameInformation data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.FrameInformation src, ihmc_common_msgs.msg.dds.FrameInformation dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FrameInformationPubSubType newInstance()
   {
      return new FrameInformationPubSubType();
   }
}

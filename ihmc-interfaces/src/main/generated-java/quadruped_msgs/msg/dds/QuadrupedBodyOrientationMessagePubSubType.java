package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedBodyOrientationMessage" defined in "QuadrupedBodyOrientationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedBodyOrientationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedBodyOrientationMessage_.idl instead.
*
*/
public class QuadrupedBodyOrientationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedBodyOrientationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "90ef4c25f355180737be6af0981e700917fb02f6a2a743207e8eed80fcb96ee0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getSo3Trajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getIsExpressedInAbsoluteTime());

      cdr.write_type_7(data.getIsAnOffsetOrientation());

      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.write(data.getSo3Trajectory(), cdr);
   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setIsExpressedInAbsoluteTime(cdr.read_type_7());
      	
      data.setIsAnOffsetOrientation(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.read(data.getSo3Trajectory(), cdr);	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("is_expressed_in_absolute_time", data.getIsExpressedInAbsoluteTime());
      ser.write_type_7("is_an_offset_orientation", data.getIsAnOffsetOrientation());
      ser.write_type_a("so3_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setIsExpressedInAbsoluteTime(ser.read_type_7("is_expressed_in_absolute_time"));
      data.setIsAnOffsetOrientation(ser.read_type_7("is_an_offset_orientation"));
      ser.read_type_a("so3_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage src, quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage src, quadruped_msgs.msg.dds.QuadrupedBodyOrientationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedBodyOrientationMessagePubSubType newInstance()
   {
      return new QuadrupedBodyOrientationMessagePubSubType();
   }
}

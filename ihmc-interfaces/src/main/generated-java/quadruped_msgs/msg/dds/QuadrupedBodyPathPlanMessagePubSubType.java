package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedBodyPathPlanMessage" defined in "QuadrupedBodyPathPlanMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedBodyPathPlanMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedBodyPathPlanMessage_.idl instead.
*
*/
public class QuadrupedBodyPathPlanMessagePubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedBodyPathPlanMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "50d29888c505ec813d701afc459e9d3672f2a1096214a5a361db0658a905e888";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getBodyPathPoints().size(); ++i0)
      {
          current_alignment += ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessagePubSubType.getCdrSerializedSize(data.getBodyPathPoints().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getIsExpressedInAbsoluteTime());

      if(data.getBodyPathPoints().size() <= 50)
      cdr.write_type_e(data.getBodyPathPoints());else
          throw new RuntimeException("body_path_points field exceeds the maximum length");

   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setIsExpressedInAbsoluteTime(cdr.read_type_7());
      	
      cdr.read_type_e(data.getBodyPathPoints());	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("is_expressed_in_absolute_time", data.getIsExpressedInAbsoluteTime());
      ser.write_type_e("body_path_points", data.getBodyPathPoints());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setIsExpressedInAbsoluteTime(ser.read_type_7("is_expressed_in_absolute_time"));
      ser.read_type_e("body_path_points", data.getBodyPathPoints());
   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage src, quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage src, quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedBodyPathPlanMessagePubSubType newInstance()
   {
      return new QuadrupedBodyPathPlanMessagePubSubType();
   }
}

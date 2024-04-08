package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8ParametersMessage" defined in "YOLOv8ParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8ParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8ParametersMessage_.idl instead.
*
*/
public class YOLOv8ParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8ParametersMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8ParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "85922194f776f2eadae82de9c7b57041b7156864458f8f9f00d8529f8ac096bf";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8ParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8ParametersMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8ParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_5(data.getConfidenceThreshold());

      cdr.write_type_5(data.getNonMaximumSuppressionThreshold());

      cdr.write_type_5(data.getSegmentationThreshold());

      cdr.write_type_5(data.getCandidateAcceptanceThreshold());

   }

   public static void read(perception_msgs.msg.dds.YOLOv8ParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setConfidenceThreshold(cdr.read_type_5());
      	
      data.setNonMaximumSuppressionThreshold(cdr.read_type_5());
      	
      data.setSegmentationThreshold(cdr.read_type_5());
      	
      data.setCandidateAcceptanceThreshold(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8ParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_5("confidence_threshold", data.getConfidenceThreshold());
      ser.write_type_5("non_maximum_suppression_threshold", data.getNonMaximumSuppressionThreshold());
      ser.write_type_5("segmentation_threshold", data.getSegmentationThreshold());
      ser.write_type_5("candidate_acceptance_threshold", data.getCandidateAcceptanceThreshold());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8ParametersMessage data)
   {
      data.setConfidenceThreshold(ser.read_type_5("confidence_threshold"));
      data.setNonMaximumSuppressionThreshold(ser.read_type_5("non_maximum_suppression_threshold"));
      data.setSegmentationThreshold(ser.read_type_5("segmentation_threshold"));
      data.setCandidateAcceptanceThreshold(ser.read_type_5("candidate_acceptance_threshold"));
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8ParametersMessage src, perception_msgs.msg.dds.YOLOv8ParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8ParametersMessage createData()
   {
      return new perception_msgs.msg.dds.YOLOv8ParametersMessage();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8ParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8ParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8ParametersMessage src, perception_msgs.msg.dds.YOLOv8ParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8ParametersMessagePubSubType newInstance()
   {
      return new YOLOv8ParametersMessagePubSubType();
   }
}

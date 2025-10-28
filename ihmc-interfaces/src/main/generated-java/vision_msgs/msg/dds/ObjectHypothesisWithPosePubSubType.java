package vision_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ObjectHypothesisWithPose" defined in "ObjectHypothesisWithPose_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ObjectHypothesisWithPose_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ObjectHypothesisWithPose_.idl instead.
*
*/
public class ObjectHypothesisWithPosePubSubType implements us.ihmc.pubsub.TopicDataType<vision_msgs.msg.dds.ObjectHypothesisWithPose>
{
   public static final java.lang.String name = "vision_msgs::msg::dds_::ObjectHypothesisWithPose_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "98abfbd3bee3d5ca4e2be6ea4c8979a105da342b55194cc397554849e39ecb5e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(vision_msgs.msg.dds.ObjectHypothesisWithPose data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, vision_msgs.msg.dds.ObjectHypothesisWithPose data) throws java.io.IOException
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

      current_alignment += vision_msgs.msg.dds.ObjectHypothesisPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PoseWithCovariancePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.ObjectHypothesisWithPose data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(vision_msgs.msg.dds.ObjectHypothesisWithPose data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += vision_msgs.msg.dds.ObjectHypothesisPubSubType.getCdrSerializedSize(data.getHypothesis(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PoseWithCovariancePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(vision_msgs.msg.dds.ObjectHypothesisWithPose data, us.ihmc.idl.CDR cdr)
   {
      vision_msgs.msg.dds.ObjectHypothesisPubSubType.write(data.getHypothesis(), cdr);
      geometry_msgs.msg.dds.PoseWithCovariancePubSubType.write(data.getPose(), cdr);
   }

   public static void read(vision_msgs.msg.dds.ObjectHypothesisWithPose data, us.ihmc.idl.CDR cdr)
   {
      vision_msgs.msg.dds.ObjectHypothesisPubSubType.read(data.getHypothesis(), cdr);	
      geometry_msgs.msg.dds.PoseWithCovariancePubSubType.read(data.getPose(), cdr);	

   }

   @Override
   public final void serialize(vision_msgs.msg.dds.ObjectHypothesisWithPose data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("hypothesis", new vision_msgs.msg.dds.ObjectHypothesisPubSubType(), data.getHypothesis());

      ser.write_type_a("pose", new geometry_msgs.msg.dds.PoseWithCovariancePubSubType(), data.getPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, vision_msgs.msg.dds.ObjectHypothesisWithPose data)
   {
      ser.read_type_a("hypothesis", new vision_msgs.msg.dds.ObjectHypothesisPubSubType(), data.getHypothesis());

      ser.read_type_a("pose", new geometry_msgs.msg.dds.PoseWithCovariancePubSubType(), data.getPose());

   }

   public static void staticCopy(vision_msgs.msg.dds.ObjectHypothesisWithPose src, vision_msgs.msg.dds.ObjectHypothesisWithPose dest)
   {
      dest.set(src);
   }

   @Override
   public vision_msgs.msg.dds.ObjectHypothesisWithPose createData()
   {
      return new vision_msgs.msg.dds.ObjectHypothesisWithPose();
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
   
   public void serialize(vision_msgs.msg.dds.ObjectHypothesisWithPose data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(vision_msgs.msg.dds.ObjectHypothesisWithPose data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(vision_msgs.msg.dds.ObjectHypothesisWithPose src, vision_msgs.msg.dds.ObjectHypothesisWithPose dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ObjectHypothesisWithPosePubSubType newInstance()
   {
      return new ObjectHypothesisWithPosePubSubType();
   }
}

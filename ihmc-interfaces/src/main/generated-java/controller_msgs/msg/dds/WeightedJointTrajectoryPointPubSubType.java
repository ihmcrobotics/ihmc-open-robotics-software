package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "WeightedJointTrajectoryPoint" defined in "WeightedJointTrajectoryPoint_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from WeightedJointTrajectoryPoint_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit WeightedJointTrajectoryPoint_.idl instead.
 */
public class WeightedJointTrajectoryPointPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WeightedJointTrajectoryPoint>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WeightedJointTrajectoryPoint_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public WeightedJointTrajectoryPointPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType.getCdrSerializedSize(data.getPoint(), current_alignment);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data, us.ihmc.idl.CDR cdr)
   {

      trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType.write(data.getPoint(), cdr);

      cdr.write_type_6(data.getWeight());
   }

   public static void read(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data, us.ihmc.idl.CDR cdr)
   {

      trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType.read(data.getPoint(), cdr);

      data.setWeight(cdr.read_type_6());
   }

   public static void staticCopy(controller_msgs.msg.dds.WeightedJointTrajectoryPoint src, controller_msgs.msg.dds.WeightedJointTrajectoryPoint dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WeightedJointTrajectoryPoint data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("point", new trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType(), data.getPoint());

      ser.write_type_6("weight", data.getWeight());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WeightedJointTrajectoryPoint data)
   {
      ser.read_type_a("point", new trajectory_msgs.msg.dds.JointTrajectoryPointPubSubType(), data.getPoint());

      data.setWeight(ser.read_type_6("weight"));
   }

   @Override
   public controller_msgs.msg.dds.WeightedJointTrajectoryPoint createData()
   {
      return new controller_msgs.msg.dds.WeightedJointTrajectoryPoint();
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

   public void serialize(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WeightedJointTrajectoryPoint data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.WeightedJointTrajectoryPoint src, controller_msgs.msg.dds.WeightedJointTrajectoryPoint dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WeightedJointTrajectoryPointPubSubType newInstance()
   {
      return new WeightedJointTrajectoryPointPubSubType();
   }
}
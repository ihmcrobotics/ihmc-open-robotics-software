package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "Wrench" defined in "Wrench_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from Wrench_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit Wrench_.idl instead.
 */
public class WrenchPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.Wrench>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::Wrench_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public WrenchPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.Wrench data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.Wrench data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getLinear_part(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAngular_part(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.Wrench data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getLinear_part(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAngular_part(), cdr);
   }

   public static void read(controller_msgs.msg.dds.Wrench data, us.ihmc.idl.CDR cdr)
   {

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getLinear_part(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAngular_part(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.Wrench src, controller_msgs.msg.dds.Wrench dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.Wrench data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.Wrench data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.Wrench data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("linear_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinear_part());

      ser.write_type_a("angular_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngular_part());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.Wrench data)
   {
      ser.read_type_a("linear_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinear_part());

      ser.read_type_a("angular_part", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngular_part());
   }

   @Override
   public controller_msgs.msg.dds.Wrench createData()
   {
      return new controller_msgs.msg.dds.Wrench();
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

   public void serialize(controller_msgs.msg.dds.Wrench data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.Wrench data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.Wrench src, controller_msgs.msg.dds.Wrench dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WrenchPubSubType newInstance()
   {
      return new WrenchPubSubType();
   }
}
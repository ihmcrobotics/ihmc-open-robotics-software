package controller_msgs.msg.dds;

/**
 * 
 * Topic data type of the struct "ManipulationAbortedStatus" defined in
 * "ManipulationAbortedStatus_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from ManipulationAbortedStatus_.idl by
 * us.ihmc.idl.generator.IDLGenerator. Do not update this file directly, edit
 * ManipulationAbortedStatus_.idl instead.
 *
 */
public class ManipulationAbortedStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ManipulationAbortedStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ManipulationAbortedStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ManipulationAbortedStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ManipulationAbortedStatus data)
         throws java.io.IOException
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

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ManipulationAbortedStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ManipulationAbortedStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ManipulationAbortedStatus data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ManipulationAbortedStatus data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ManipulationAbortedStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ManipulationAbortedStatus data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());
   }

   public static void staticCopy(controller_msgs.msg.dds.ManipulationAbortedStatus src, controller_msgs.msg.dds.ManipulationAbortedStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ManipulationAbortedStatus createData()
   {
      return new controller_msgs.msg.dds.ManipulationAbortedStatus();
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

   public void serialize(controller_msgs.msg.dds.ManipulationAbortedStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ManipulationAbortedStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.ManipulationAbortedStatus src, controller_msgs.msg.dds.ManipulationAbortedStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ManipulationAbortedStatusPubSubType newInstance()
   {
      return new ManipulationAbortedStatusPubSubType();
   }
}

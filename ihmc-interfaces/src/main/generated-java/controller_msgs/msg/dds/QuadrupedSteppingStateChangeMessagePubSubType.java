package controller_msgs.msg.dds;

/**
 *
 * Topic data type of the struct "QuadrupedSteppingStateChangeMessage" defined in "QuadrupedSteppingStateChangeMessage_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from QuadrupedSteppingStateChangeMessage_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit QuadrupedSteppingStateChangeMessage_.idl instead.
 *
 */
public class QuadrupedSteppingStateChangeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedSteppingStateChangeMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data)
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getInitialSteppingControllerName());

      cdr.write_type_9(data.getEndSteppingControllerName());

   }

   public static void read(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setInitialSteppingControllerName(cdr.read_type_9());

      data.setEndSteppingControllerName(cdr.read_type_9());

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("initial_stepping_controller_name", data.getInitialSteppingControllerName());
      ser.write_type_9("end_stepping_controller_name", data.getEndSteppingControllerName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data)
   {
      data.setInitialSteppingControllerName(ser.read_type_9("initial_stepping_controller_name"));
      data.setEndSteppingControllerName(ser.read_type_9("end_stepping_controller_name"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage src,
                                 controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage createData()
   {
      return new controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage();
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

   public void serialize(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage src, controller_msgs.msg.dds.QuadrupedSteppingStateChangeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedSteppingStateChangeMessagePubSubType newInstance()
   {
      return new QuadrupedSteppingStateChangeMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuixSideStepDirectionMessage" defined in "QuixSideStepDirectionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuixSideStepDirectionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuixSideStepDirectionMessage_.idl instead.
*
*/
public class QuixSideStepDirectionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuixSideStepDirectionMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuixSideStepDirectionMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuixSideStepDirectionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuixSideStepDirectionMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuixSideStepDirectionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuixSideStepDirectionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuixSideStepDirectionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getStepDirection());

   }

   public static void read(controller_msgs.msg.dds.QuixSideStepDirectionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setStepDirection(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuixSideStepDirectionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("step_direction", data.getStepDirection());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuixSideStepDirectionMessage data)
   {
      data.setStepDirection(ser.read_type_9("step_direction"));   }

   public static void staticCopy(controller_msgs.msg.dds.QuixSideStepDirectionMessage src, controller_msgs.msg.dds.QuixSideStepDirectionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuixSideStepDirectionMessage createData()
   {
      return new controller_msgs.msg.dds.QuixSideStepDirectionMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuixSideStepDirectionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuixSideStepDirectionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuixSideStepDirectionMessage src, controller_msgs.msg.dds.QuixSideStepDirectionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuixSideStepDirectionMessagePubSubType newInstance()
   {
      return new QuixSideStepDirectionMessagePubSubType();
   }
}

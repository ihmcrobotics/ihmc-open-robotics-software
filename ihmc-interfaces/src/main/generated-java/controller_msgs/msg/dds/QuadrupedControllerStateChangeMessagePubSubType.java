package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedControllerStateChangeMessage" defined in "QuadrupedControllerStateChangeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedControllerStateChangeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedControllerStateChangeMessage_.idl instead.
*
*/
public class QuadrupedControllerStateChangeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QuadrupedControllerStateChangeMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getInitialControllerName());

      cdr.write_type_9(data.getEndControllerName());

   }

   public static void read(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setInitialControllerName(cdr.read_type_9());
      	
      data.setEndControllerName(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("initial_controller_name", data.getInitialControllerName());
      ser.write_type_9("end_controller_name", data.getEndControllerName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data)
   {
      data.setInitialControllerName(ser.read_type_9("initial_controller_name"));
      data.setEndControllerName(ser.read_type_9("end_controller_name"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage src, controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage createData()
   {
      return new controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage src, controller_msgs.msg.dds.QuadrupedControllerStateChangeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedControllerStateChangeMessagePubSubType newInstance()
   {
      return new QuadrupedControllerStateChangeMessagePubSubType();
   }
}

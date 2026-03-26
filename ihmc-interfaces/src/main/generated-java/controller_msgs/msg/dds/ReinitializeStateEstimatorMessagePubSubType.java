package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ReinitializeStateEstimatorMessage" defined in "ReinitializeStateEstimatorMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ReinitializeStateEstimatorMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ReinitializeStateEstimatorMessage_.idl instead.
*
*/
public class ReinitializeStateEstimatorMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ReinitializeStateEstimatorMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ReinitializeStateEstimatorMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "50831cd8f159f9214bba3962c405ab92405e24d2944a27e0d89714c5083726e9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getRequestReinitialize());

   }

   public static void read(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRequestReinitialize(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("request_reinitialize", data.getRequestReinitialize());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data)
   {
      data.setRequestReinitialize(ser.read_type_7("request_reinitialize"));   }

   public static void staticCopy(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage src, controller_msgs.msg.dds.ReinitializeStateEstimatorMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ReinitializeStateEstimatorMessage createData()
   {
      return new controller_msgs.msg.dds.ReinitializeStateEstimatorMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ReinitializeStateEstimatorMessage src, controller_msgs.msg.dds.ReinitializeStateEstimatorMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ReinitializeStateEstimatorMessagePubSubType newInstance()
   {
      return new ReinitializeStateEstimatorMessagePubSubType();
   }
}

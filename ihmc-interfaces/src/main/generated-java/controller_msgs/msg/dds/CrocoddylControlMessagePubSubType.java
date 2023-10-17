package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CrocoddylControlMessage" defined in "CrocoddylControlMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CrocoddylControlMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CrocoddylControlMessage_.idl instead.
*
*/
public class CrocoddylControlMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CrocoddylControlMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CrocoddylControlMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c642e0d56113b1287f8cc1650faed7773062dd5ce572edf8ec12eb25932f362f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CrocoddylControlMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CrocoddylControlMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.CrocoddylFeedbackGainMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylControlMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylControlMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getU().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.CrocoddylFeedbackGainMessagePubSubType.getCdrSerializedSize(data.getGain(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CrocoddylControlMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getInput());

      cdr.write_type_9(data.getParametrization());

      if(data.getU().size() <= 100)
      cdr.write_type_e(data.getU());else
          throw new RuntimeException("u field exceeds the maximum length");

      controller_msgs.msg.dds.CrocoddylFeedbackGainMessagePubSubType.write(data.getGain(), cdr);
   }

   public static void read(controller_msgs.msg.dds.CrocoddylControlMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setInput(cdr.read_type_9());
      	
      data.setParametrization(cdr.read_type_9());
      	
      cdr.read_type_e(data.getU());	
      controller_msgs.msg.dds.CrocoddylFeedbackGainMessagePubSubType.read(data.getGain(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CrocoddylControlMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("input", data.getInput());
      ser.write_type_9("parametrization", data.getParametrization());
      ser.write_type_e("u", data.getU());
      ser.write_type_a("gain", new controller_msgs.msg.dds.CrocoddylFeedbackGainMessagePubSubType(), data.getGain());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CrocoddylControlMessage data)
   {
      data.setInput(ser.read_type_9("input"));
      data.setParametrization(ser.read_type_9("parametrization"));
      ser.read_type_e("u", data.getU());
      ser.read_type_a("gain", new controller_msgs.msg.dds.CrocoddylFeedbackGainMessagePubSubType(), data.getGain());

   }

   public static void staticCopy(controller_msgs.msg.dds.CrocoddylControlMessage src, controller_msgs.msg.dds.CrocoddylControlMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CrocoddylControlMessage createData()
   {
      return new controller_msgs.msg.dds.CrocoddylControlMessage();
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
   
   public void serialize(controller_msgs.msg.dds.CrocoddylControlMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CrocoddylControlMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CrocoddylControlMessage src, controller_msgs.msg.dds.CrocoddylControlMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CrocoddylControlMessagePubSubType newInstance()
   {
      return new CrocoddylControlMessagePubSubType();
   }
}

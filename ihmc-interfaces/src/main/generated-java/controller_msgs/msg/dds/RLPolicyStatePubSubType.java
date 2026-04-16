package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RLPolicyState" defined in "RLPolicyState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RLPolicyState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RLPolicyState_.idl instead.
*
*/
public class RLPolicyStatePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RLPolicyState>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RLPolicyState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "44a6a26cb5372741db5005264ad7196eeb043ac509d9e6aee8af5f419de56068";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RLPolicyState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RLPolicyState data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RLPolicyState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RLPolicyState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAvailableModels().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getAvailableModels().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RLPolicyState data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getCurrentModel());

      cdr.write_type_9(data.getNumAvailableModels());

      if(data.getAvailableModels().size() <= 16)
      cdr.write_type_e(data.getAvailableModels());else
          throw new RuntimeException("available_models field exceeds the maximum length: %d > %d".formatted(data.getAvailableModels().size(), 16));

   }

   public static void read(controller_msgs.msg.dds.RLPolicyState data, us.ihmc.idl.CDR cdr)
   {
      data.setCurrentModel(cdr.read_type_9());
      	
      data.setNumAvailableModels(cdr.read_type_9());
      	
      cdr.read_type_e(data.getAvailableModels());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RLPolicyState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("current_model", data.getCurrentModel());
      ser.write_type_9("num_available_models", data.getNumAvailableModels());
      ser.write_type_e("available_models", data.getAvailableModels());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RLPolicyState data)
   {
      data.setCurrentModel(ser.read_type_9("current_model"));
      data.setNumAvailableModels(ser.read_type_9("num_available_models"));
      ser.read_type_e("available_models", data.getAvailableModels());
   }

   public static void staticCopy(controller_msgs.msg.dds.RLPolicyState src, controller_msgs.msg.dds.RLPolicyState dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RLPolicyState createData()
   {
      return new controller_msgs.msg.dds.RLPolicyState();
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
   
   public void serialize(controller_msgs.msg.dds.RLPolicyState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RLPolicyState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RLPolicyState src, controller_msgs.msg.dds.RLPolicyState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RLPolicyStatePubSubType newInstance()
   {
      return new RLPolicyStatePubSubType();
   }
}

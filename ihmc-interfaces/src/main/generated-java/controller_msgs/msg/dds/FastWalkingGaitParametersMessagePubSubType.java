package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FastWalkingGaitParametersMessage" defined in "FastWalkingGaitParametersMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FastWalkingGaitParametersMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FastWalkingGaitParametersMessage_.idl instead.
*
*/
public class FastWalkingGaitParametersMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FastWalkingGaitParametersMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FastWalkingGaitParametersMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "49a387481b051b3039a2a70a6f72e5f857fdb61939a51d902e936c4e96a7e936";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FastWalkingGaitParametersMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getSwingHeight());

      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getDoubleSupportFraction());

   }

   public static void read(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSwingHeight(cdr.read_type_6());
      	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setDoubleSupportFraction(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("swing_height", data.getSwingHeight());
      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("double_support_fraction", data.getDoubleSupportFraction());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FastWalkingGaitParametersMessage data)
   {
      data.setSwingHeight(ser.read_type_6("swing_height"));
      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setDoubleSupportFraction(ser.read_type_6("double_support_fraction"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FastWalkingGaitParametersMessage src, controller_msgs.msg.dds.FastWalkingGaitParametersMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FastWalkingGaitParametersMessage createData()
   {
      return new controller_msgs.msg.dds.FastWalkingGaitParametersMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FastWalkingGaitParametersMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FastWalkingGaitParametersMessage src, controller_msgs.msg.dds.FastWalkingGaitParametersMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FastWalkingGaitParametersMessagePubSubType newInstance()
   {
      return new FastWalkingGaitParametersMessagePubSubType();
   }
}

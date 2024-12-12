package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PID3DGains" defined in "PID3DGains_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PID3DGains_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PID3DGains_.idl instead.
*
*/
public class PID3DGainsPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PID3DGains>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PID3DGains_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d4da515998705e0bd9e95e5b3676322e8e22957f12aba60eeaa7349227a66fe5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PID3DGains data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PID3DGains data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.PIDGainsPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.PIDGainsPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.PIDGainsPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PID3DGains data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PID3DGains data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += controller_msgs.msg.dds.PIDGainsPubSubType.getCdrSerializedSize(data.getGainsX(), current_alignment);

      current_alignment += controller_msgs.msg.dds.PIDGainsPubSubType.getCdrSerializedSize(data.getGainsY(), current_alignment);

      current_alignment += controller_msgs.msg.dds.PIDGainsPubSubType.getCdrSerializedSize(data.getGainsZ(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PID3DGains data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.PIDGainsPubSubType.write(data.getGainsX(), cdr);
      controller_msgs.msg.dds.PIDGainsPubSubType.write(data.getGainsY(), cdr);
      controller_msgs.msg.dds.PIDGainsPubSubType.write(data.getGainsZ(), cdr);
      cdr.write_type_6(data.getMaximumFeedback());

      cdr.write_type_6(data.getMaximumFeedbackRate());

      cdr.write_type_6(data.getMaximumIntegralError());

      cdr.write_type_6(data.getMaximumDerivativeError());

      cdr.write_type_6(data.getMaximumProportionalError());

   }

   public static void read(controller_msgs.msg.dds.PID3DGains data, us.ihmc.idl.CDR cdr)
   {
      controller_msgs.msg.dds.PIDGainsPubSubType.read(data.getGainsX(), cdr);	
      controller_msgs.msg.dds.PIDGainsPubSubType.read(data.getGainsY(), cdr);	
      controller_msgs.msg.dds.PIDGainsPubSubType.read(data.getGainsZ(), cdr);	
      data.setMaximumFeedback(cdr.read_type_6());
      	
      data.setMaximumFeedbackRate(cdr.read_type_6());
      	
      data.setMaximumIntegralError(cdr.read_type_6());
      	
      data.setMaximumDerivativeError(cdr.read_type_6());
      	
      data.setMaximumProportionalError(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PID3DGains data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("gains_x", new controller_msgs.msg.dds.PIDGainsPubSubType(), data.getGainsX());

      ser.write_type_a("gains_y", new controller_msgs.msg.dds.PIDGainsPubSubType(), data.getGainsY());

      ser.write_type_a("gains_z", new controller_msgs.msg.dds.PIDGainsPubSubType(), data.getGainsZ());

      ser.write_type_6("maximum_feedback", data.getMaximumFeedback());
      ser.write_type_6("maximum_feedback_rate", data.getMaximumFeedbackRate());
      ser.write_type_6("maximum_integral_error", data.getMaximumIntegralError());
      ser.write_type_6("maximum_derivative_error", data.getMaximumDerivativeError());
      ser.write_type_6("maximum_proportional_error", data.getMaximumProportionalError());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PID3DGains data)
   {
      ser.read_type_a("gains_x", new controller_msgs.msg.dds.PIDGainsPubSubType(), data.getGainsX());

      ser.read_type_a("gains_y", new controller_msgs.msg.dds.PIDGainsPubSubType(), data.getGainsY());

      ser.read_type_a("gains_z", new controller_msgs.msg.dds.PIDGainsPubSubType(), data.getGainsZ());

      data.setMaximumFeedback(ser.read_type_6("maximum_feedback"));
      data.setMaximumFeedbackRate(ser.read_type_6("maximum_feedback_rate"));
      data.setMaximumIntegralError(ser.read_type_6("maximum_integral_error"));
      data.setMaximumDerivativeError(ser.read_type_6("maximum_derivative_error"));
      data.setMaximumProportionalError(ser.read_type_6("maximum_proportional_error"));
   }

   public static void staticCopy(controller_msgs.msg.dds.PID3DGains src, controller_msgs.msg.dds.PID3DGains dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PID3DGains createData()
   {
      return new controller_msgs.msg.dds.PID3DGains();
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
   
   public void serialize(controller_msgs.msg.dds.PID3DGains data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PID3DGains data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PID3DGains src, controller_msgs.msg.dds.PID3DGains dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PID3DGainsPubSubType newInstance()
   {
      return new PID3DGainsPubSubType();
   }
}

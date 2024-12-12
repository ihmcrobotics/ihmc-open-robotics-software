package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SE3PIDGainsTrajectoryPointMessage" defined in "SE3PIDGainsTrajectoryPointMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SE3PIDGainsTrajectoryPointMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SE3PIDGainsTrajectoryPointMessage_.idl instead.
*
*/
public class SE3PIDGainsTrajectoryPointMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SE3PIDGainsTrajectoryPointMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e04cdc111cd7487d07550d375bb203fa8ebd0e8dd045e829d11c40569dbad462";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.PID3DGainsPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.PID3DGainsPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.PID3DGainsPubSubType.getCdrSerializedSize(data.getLinearGains(), current_alignment);

      current_alignment += controller_msgs.msg.dds.PID3DGainsPubSubType.getCdrSerializedSize(data.getAngularGains(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getTime());

      controller_msgs.msg.dds.PID3DGainsPubSubType.write(data.getLinearGains(), cdr);
      controller_msgs.msg.dds.PID3DGainsPubSubType.write(data.getAngularGains(), cdr);
   }

   public static void read(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTime(cdr.read_type_6());
      	
      controller_msgs.msg.dds.PID3DGainsPubSubType.read(data.getLinearGains(), cdr);	
      controller_msgs.msg.dds.PID3DGainsPubSubType.read(data.getAngularGains(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("time", data.getTime());
      ser.write_type_a("linear_gains", new controller_msgs.msg.dds.PID3DGainsPubSubType(), data.getLinearGains());

      ser.write_type_a("angular_gains", new controller_msgs.msg.dds.PID3DGainsPubSubType(), data.getAngularGains());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTime(ser.read_type_6("time"));
      ser.read_type_a("linear_gains", new controller_msgs.msg.dds.PID3DGainsPubSubType(), data.getLinearGains());

      ser.read_type_a("angular_gains", new controller_msgs.msg.dds.PID3DGainsPubSubType(), data.getAngularGains());

   }

   public static void staticCopy(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage src, controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage createData()
   {
      return new controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage src, controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SE3PIDGainsTrajectoryPointMessagePubSubType newInstance()
   {
      return new SE3PIDGainsTrajectoryPointMessagePubSubType();
   }
}

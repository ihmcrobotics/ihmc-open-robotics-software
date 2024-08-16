package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SO3StreamingMessage" defined in "SO3StreamingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SO3StreamingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SO3StreamingMessage_.idl instead.
*
*/
public class SO3StreamingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.SO3StreamingMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::SO3StreamingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "97f6a848dacda67d6b1236d71154a2906194853496c76d2022d76a0e31432012";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.SO3StreamingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.SO3StreamingMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.FrameInformationPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SO3StreamingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SO3StreamingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.FrameInformationPubSubType.getCdrSerializedSize(data.getFrameInformation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getControlFramePose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAngularVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAngularAcceleration(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.SO3StreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.write(data.getFrameInformation(), cdr);
      cdr.write_type_7(data.getUseCustomControlFrame());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getControlFramePose(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAngularVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAngularAcceleration(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.SO3StreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.FrameInformationPubSubType.read(data.getFrameInformation(), cdr);	
      data.setUseCustomControlFrame(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getControlFramePose(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAngularVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAngularAcceleration(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.SO3StreamingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("frame_information", new ihmc_common_msgs.msg.dds.FrameInformationPubSubType(), data.getFrameInformation());

      ser.write_type_7("use_custom_control_frame", data.getUseCustomControlFrame());
      ser.write_type_a("control_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getControlFramePose());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_a("angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularVelocity());

      ser.write_type_a("angular_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularAcceleration());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.SO3StreamingMessage data)
   {
      ser.read_type_a("frame_information", new ihmc_common_msgs.msg.dds.FrameInformationPubSubType(), data.getFrameInformation());

      data.setUseCustomControlFrame(ser.read_type_7("use_custom_control_frame"));
      ser.read_type_a("control_frame_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getControlFramePose());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.read_type_a("angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularVelocity());

      ser.read_type_a("angular_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularAcceleration());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.SO3StreamingMessage src, ihmc_common_msgs.msg.dds.SO3StreamingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.SO3StreamingMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.SO3StreamingMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.SO3StreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.SO3StreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.SO3StreamingMessage src, ihmc_common_msgs.msg.dds.SO3StreamingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SO3StreamingMessagePubSubType newInstance()
   {
      return new SO3StreamingMessagePubSubType();
   }
}

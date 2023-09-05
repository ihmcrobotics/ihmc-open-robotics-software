package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "REAStatusMessage" defined in "REAStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from REAStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit REAStatusMessage_.idl instead.
*
*/
public class REAStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.REAStatusMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::REAStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "246ff00996f4e9f2f8b62751b98aa511ac535b45f69c41e7d445c693d3448cdc";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.REAStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.REAStatusMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += perception_msgs.msg.dds.REASensorDataFilterParametersMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.REAStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.REAStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += perception_msgs.msg.dds.REASensorDataFilterParametersMessagePubSubType.getCdrSerializedSize(data.getCurrentSensorFilterParameters(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.REAStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getIsRunning());

      cdr.write_type_7(data.getIsUsingLidar());

      cdr.write_type_7(data.getIsUsingStereoVision());

      cdr.write_type_7(data.getHasCleared());

      perception_msgs.msg.dds.REASensorDataFilterParametersMessagePubSubType.write(data.getCurrentSensorFilterParameters(), cdr);
   }

   public static void read(perception_msgs.msg.dds.REAStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setIsRunning(cdr.read_type_7());
      	
      data.setIsUsingLidar(cdr.read_type_7());
      	
      data.setIsUsingStereoVision(cdr.read_type_7());
      	
      data.setHasCleared(cdr.read_type_7());
      	
      perception_msgs.msg.dds.REASensorDataFilterParametersMessagePubSubType.read(data.getCurrentSensorFilterParameters(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.REAStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("is_running", data.getIsRunning());
      ser.write_type_7("is_using_lidar", data.getIsUsingLidar());
      ser.write_type_7("is_using_stereo_vision", data.getIsUsingStereoVision());
      ser.write_type_7("has_cleared", data.getHasCleared());
      ser.write_type_a("current_sensor_filter_parameters", new perception_msgs.msg.dds.REASensorDataFilterParametersMessagePubSubType(), data.getCurrentSensorFilterParameters());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.REAStatusMessage data)
   {
      data.setIsRunning(ser.read_type_7("is_running"));
      data.setIsUsingLidar(ser.read_type_7("is_using_lidar"));
      data.setIsUsingStereoVision(ser.read_type_7("is_using_stereo_vision"));
      data.setHasCleared(ser.read_type_7("has_cleared"));
      ser.read_type_a("current_sensor_filter_parameters", new perception_msgs.msg.dds.REASensorDataFilterParametersMessagePubSubType(), data.getCurrentSensorFilterParameters());

   }

   public static void staticCopy(perception_msgs.msg.dds.REAStatusMessage src, perception_msgs.msg.dds.REAStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.REAStatusMessage createData()
   {
      return new perception_msgs.msg.dds.REAStatusMessage();
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
   
   public void serialize(perception_msgs.msg.dds.REAStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.REAStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.REAStatusMessage src, perception_msgs.msg.dds.REAStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public REAStatusMessagePubSubType newInstance()
   {
      return new REAStatusMessagePubSubType();
   }
}

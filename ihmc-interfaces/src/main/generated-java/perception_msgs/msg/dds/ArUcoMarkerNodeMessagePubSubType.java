package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ArUcoMarkerNodeMessage" defined in "ArUcoMarkerNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ArUcoMarkerNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ArUcoMarkerNodeMessage_.idl instead.
*
*/
public class ArUcoMarkerNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.ArUcoMarkerNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ArUcoMarkerNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "01da7e9f97baec698acf67d654a5e1b52421097af5cda32d3d21d3fc83535ed0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.ArUcoMarkerNodeMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNode(), current_alignment);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.write(data.getDetectableSceneNode(), cdr);
      cdr.write_type_3(data.getMarkerId());

      cdr.write_type_5(data.getMarkerSize());

      cdr.write_type_5(data.getBreakFrequency());

   }

   public static void read(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.read(data.getDetectableSceneNode(), cdr);	
      data.setMarkerId(cdr.read_type_3());
      	
      data.setMarkerSize(cdr.read_type_5());
      	
      data.setBreakFrequency(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      ser.write_type_3("marker_id", data.getMarkerId());
      ser.write_type_5("marker_size", data.getMarkerSize());
      ser.write_type_5("break_frequency", data.getBreakFrequency());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ArUcoMarkerNodeMessage data)
   {
      ser.read_type_a("detectable_scene_node", new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType(), data.getDetectableSceneNode());

      data.setMarkerId(ser.read_type_3("marker_id"));
      data.setMarkerSize(ser.read_type_5("marker_size"));
      data.setBreakFrequency(ser.read_type_5("break_frequency"));
   }

   public static void staticCopy(perception_msgs.msg.dds.ArUcoMarkerNodeMessage src, perception_msgs.msg.dds.ArUcoMarkerNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.ArUcoMarkerNodeMessage createData()
   {
      return new perception_msgs.msg.dds.ArUcoMarkerNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.ArUcoMarkerNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.ArUcoMarkerNodeMessage src, perception_msgs.msg.dds.ArUcoMarkerNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ArUcoMarkerNodeMessagePubSubType newInstance()
   {
      return new ArUcoMarkerNodeMessagePubSubType();
   }
}

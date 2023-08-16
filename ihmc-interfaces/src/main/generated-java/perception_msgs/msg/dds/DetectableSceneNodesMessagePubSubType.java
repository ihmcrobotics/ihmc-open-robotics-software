package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectableSceneNodesMessage" defined in "DetectableSceneNodesMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectableSceneNodesMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectableSceneNodesMessage_.idl instead.
*
*/
public class DetectableSceneNodesMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectableSceneNodesMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectableSceneNodesMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4fc55eb6c99345a158c1671466a67cbf9e71dc49624855a00db2660080169cdd";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectableSceneNodesMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectableSceneNodesMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 5000; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneNodesMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectableSceneNodesMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getDetectableSceneNodes().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.getCdrSerializedSize(data.getDetectableSceneNodes().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectableSceneNodesMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getDetectableSceneNodes().size() <= 5000)
      cdr.write_type_e(data.getDetectableSceneNodes());else
          throw new RuntimeException("detectable_scene_nodes field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.DetectableSceneNodesMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getDetectableSceneNodes());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectableSceneNodesMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("detectable_scene_nodes", data.getDetectableSceneNodes());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectableSceneNodesMessage data)
   {
      ser.read_type_e("detectable_scene_nodes", data.getDetectableSceneNodes());
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectableSceneNodesMessage src, perception_msgs.msg.dds.DetectableSceneNodesMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectableSceneNodesMessage createData()
   {
      return new perception_msgs.msg.dds.DetectableSceneNodesMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DetectableSceneNodesMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectableSceneNodesMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectableSceneNodesMessage src, perception_msgs.msg.dds.DetectableSceneNodesMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectableSceneNodesMessagePubSubType newInstance()
   {
      return new DetectableSceneNodesMessagePubSubType();
   }
}

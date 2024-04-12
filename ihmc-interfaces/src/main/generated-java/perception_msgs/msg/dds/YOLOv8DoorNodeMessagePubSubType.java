package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8DoorNodeMessage" defined in "YOLOv8DoorNodeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8DoorNodeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8DoorNodeMessage_.idl instead.
*
*/
public class YOLOv8DoorNodeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8DoorNodeMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8DoorNodeMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "507cf04a091323bcbb25e65f7bc35939397eef005e42ecfe980dbf6e0e91333a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8DoorNodeMessage data) throws java.io.IOException
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
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPresumedDoorPointCloud().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Point32PubSubType.getCdrSerializedSize(data.getPresumedDoorPointCloud().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPresumedDoorPointCloud().size() <= 5000)
      cdr.write_type_e(data.getPresumedDoorPointCloud());else
          throw new RuntimeException("presumed_door_point_cloud field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPresumedDoorPointCloud());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("presumed_door_point_cloud", data.getPresumedDoorPointCloud());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8DoorNodeMessage data)
   {
      ser.read_type_e("presumed_door_point_cloud", data.getPresumedDoorPointCloud());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8DoorNodeMessage src, perception_msgs.msg.dds.YOLOv8DoorNodeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8DoorNodeMessage createData()
   {
      return new perception_msgs.msg.dds.YOLOv8DoorNodeMessage();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8DoorNodeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8DoorNodeMessage src, perception_msgs.msg.dds.YOLOv8DoorNodeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8DoorNodeMessagePubSubType newInstance()
   {
      return new YOLOv8DoorNodeMessagePubSubType();
   }
}

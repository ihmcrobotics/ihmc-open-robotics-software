package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RStatusMessage" defined in "AI2RStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RStatusMessage_.idl instead.
*
*/
public class AI2RStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "95faf437d195d65ffbf399e7c9a445d1e2fba56dbfc3d5917b05fbfb73ae6135";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RStatusMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.AI2RObjectMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 200; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getRobotMidFeetUnderPelvisPoseInWorld(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getObjects().size(); ++i0)
      {
          current_alignment += behavior_msgs.msg.dds.AI2RObjectMessagePubSubType.getCdrSerializedSize(data.getObjects().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAvailableBehaviors().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getAvailableBehaviors().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.write(data.getRobotMidFeetUnderPelvisPoseInWorld(), cdr);
      if(data.getObjects().size() <= 200)
      cdr.write_type_e(data.getObjects());else
          throw new RuntimeException("objects field exceeds the maximum length");

      if(data.getAvailableBehaviors().size() <= 200)
      cdr.write_type_e(data.getAvailableBehaviors());else
          throw new RuntimeException("available_behaviors field exceeds the maximum length");

   }

   public static void read(behavior_msgs.msg.dds.AI2RStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.read(data.getRobotMidFeetUnderPelvisPoseInWorld(), cdr);	
      cdr.read_type_e(data.getObjects());	
      cdr.read_type_e(data.getAvailableBehaviors());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("robot_mid_feet_under_pelvis_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getRobotMidFeetUnderPelvisPoseInWorld());

      ser.write_type_e("objects", data.getObjects());
      ser.write_type_e("available_behaviors", data.getAvailableBehaviors());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RStatusMessage data)
   {
      ser.read_type_a("robot_mid_feet_under_pelvis_pose_in_world", new geometry_msgs.msg.dds.PosePubSubType(), data.getRobotMidFeetUnderPelvisPoseInWorld());

      ser.read_type_e("objects", data.getObjects());
      ser.read_type_e("available_behaviors", data.getAvailableBehaviors());
   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RStatusMessage src, behavior_msgs.msg.dds.AI2RStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RStatusMessage src, behavior_msgs.msg.dds.AI2RStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RStatusMessagePubSubType newInstance()
   {
      return new AI2RStatusMessagePubSubType();
   }
}

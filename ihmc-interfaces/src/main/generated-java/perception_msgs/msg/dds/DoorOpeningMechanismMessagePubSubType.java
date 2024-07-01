package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DoorOpeningMechanismMessage" defined in "DoorOpeningMechanismMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DoorOpeningMechanismMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DoorOpeningMechanismMessage_.idl instead.
*
*/
public class DoorOpeningMechanismMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DoorOpeningMechanismMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DoorOpeningMechanismMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "cb84d3ee40561e49251977a1989b7ee9fa577cd919e52e4b77141ca94bd76197";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DoorOpeningMechanismMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DoorOpeningMechanismMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorOpeningMechanismMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorOpeningMechanismMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getGraspPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getType());

      cdr.write_type_9(data.getDoorSide());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getGraspPose(), cdr);
   }

   public static void read(perception_msgs.msg.dds.DoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setType(cdr.read_type_9());
      	
      data.setDoorSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getGraspPose(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DoorOpeningMechanismMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("type", data.getType());
      ser.write_type_9("door_side", data.getDoorSide());
      ser.write_type_a("grasp_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGraspPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DoorOpeningMechanismMessage data)
   {
      data.setType(ser.read_type_9("type"));
      data.setDoorSide(ser.read_type_9("door_side"));
      ser.read_type_a("grasp_pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getGraspPose());

   }

   public static void staticCopy(perception_msgs.msg.dds.DoorOpeningMechanismMessage src, perception_msgs.msg.dds.DoorOpeningMechanismMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DoorOpeningMechanismMessage createData()
   {
      return new perception_msgs.msg.dds.DoorOpeningMechanismMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DoorOpeningMechanismMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DoorOpeningMechanismMessage src, perception_msgs.msg.dds.DoorOpeningMechanismMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DoorOpeningMechanismMessagePubSubType newInstance()
   {
      return new DoorOpeningMechanismMessagePubSubType();
   }
}

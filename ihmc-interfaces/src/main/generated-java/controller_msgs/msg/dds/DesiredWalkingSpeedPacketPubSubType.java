package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DesiredWalkingSpeedPacket" defined in "DesiredWalkingSpeedPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DesiredWalkingSpeedPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DesiredWalkingSpeedPacket_.idl instead.
*
*/
public class DesiredWalkingSpeedPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DesiredWalkingSpeedPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DesiredWalkingSpeedPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DesiredWalkingSpeedPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredSpeed(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredSpeed(), cdr);
      cdr.write_type_6(data.getTurningSpeed());

   }

   public static void read(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredSpeed(), cdr);	
      data.setTurningSpeed(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("desired_speed", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredSpeed());

      ser.write_type_6("turning_speed", data.getTurningSpeed());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DesiredWalkingSpeedPacket data)
   {
      ser.read_type_a("desired_speed", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredSpeed());

      data.setTurningSpeed(ser.read_type_6("turning_speed"));
   }

   public static void staticCopy(controller_msgs.msg.dds.DesiredWalkingSpeedPacket src, controller_msgs.msg.dds.DesiredWalkingSpeedPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.DesiredWalkingSpeedPacket createData()
   {
      return new controller_msgs.msg.dds.DesiredWalkingSpeedPacket();
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
   
   public void serialize(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DesiredWalkingSpeedPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.DesiredWalkingSpeedPacket src, controller_msgs.msg.dds.DesiredWalkingSpeedPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DesiredWalkingSpeedPacketPubSubType newInstance()
   {
      return new DesiredWalkingSpeedPacketPubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BDIBehaviorCommandPacket" defined in "BDIBehaviorCommandPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BDIBehaviorCommandPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BDIBehaviorCommandPacket_.idl instead.
*
*/
public class BDIBehaviorCommandPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BDIBehaviorCommandPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BDIBehaviorCommandPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BDIBehaviorCommandPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BDIBehaviorCommandPacket data) throws java.io.IOException
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


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BDIBehaviorCommandPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BDIBehaviorCommandPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BDIBehaviorCommandPacket data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_9(data.getAtlasBdiRobotBehavior());


      cdr.write_type_7(data.getStop());

   }

   public static void read(controller_msgs.msg.dds.BDIBehaviorCommandPacket data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setAtlasBdiRobotBehavior(cdr.read_type_9());
      	

      data.setStop(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BDIBehaviorCommandPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_9("atlas_bdi_robot_behavior", data.getAtlasBdiRobotBehavior());

      ser.write_type_7("stop", data.getStop());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BDIBehaviorCommandPacket data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setAtlasBdiRobotBehavior(ser.read_type_9("atlas_bdi_robot_behavior"));

      data.setStop(ser.read_type_7("stop"));
   }

   public static void staticCopy(controller_msgs.msg.dds.BDIBehaviorCommandPacket src, controller_msgs.msg.dds.BDIBehaviorCommandPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BDIBehaviorCommandPacket createData()
   {
      return new controller_msgs.msg.dds.BDIBehaviorCommandPacket();
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
   
   public void serialize(controller_msgs.msg.dds.BDIBehaviorCommandPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BDIBehaviorCommandPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BDIBehaviorCommandPacket src, controller_msgs.msg.dds.BDIBehaviorCommandPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BDIBehaviorCommandPacketPubSubType newInstance()
   {
      return new BDIBehaviorCommandPacketPubSubType();
   }
}

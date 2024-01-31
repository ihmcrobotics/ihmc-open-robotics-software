package atlas_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BDIBehaviorStatusPacket" defined in "BDIBehaviorStatusPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BDIBehaviorStatusPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BDIBehaviorStatusPacket_.idl instead.
*
*/
public class BDIBehaviorStatusPacketPubSubType implements us.ihmc.pubsub.TopicDataType<atlas_msgs.msg.dds.BDIBehaviorStatusPacket>
{
   public static final java.lang.String name = "atlas_msgs::msg::dds_::BDIBehaviorStatusPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d69b22c5754e0d15e681b644f8af90284984346c8867b06566a921d9399caf92";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, atlas_msgs.msg.dds.BDIBehaviorStatusPacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getCurrentBdiRobotBehavior());

   }

   public static void read(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCurrentBdiRobotBehavior(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("current_bdi_robot_behavior", data.getCurrentBdiRobotBehavior());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, atlas_msgs.msg.dds.BDIBehaviorStatusPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCurrentBdiRobotBehavior(ser.read_type_9("current_bdi_robot_behavior"));
   }

   public static void staticCopy(atlas_msgs.msg.dds.BDIBehaviorStatusPacket src, atlas_msgs.msg.dds.BDIBehaviorStatusPacket dest)
   {
      dest.set(src);
   }

   @Override
   public atlas_msgs.msg.dds.BDIBehaviorStatusPacket createData()
   {
      return new atlas_msgs.msg.dds.BDIBehaviorStatusPacket();
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
   
   public void serialize(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(atlas_msgs.msg.dds.BDIBehaviorStatusPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(atlas_msgs.msg.dds.BDIBehaviorStatusPacket src, atlas_msgs.msg.dds.BDIBehaviorStatusPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BDIBehaviorStatusPacketPubSubType newInstance()
   {
      return new BDIBehaviorStatusPacketPubSubType();
   }
}

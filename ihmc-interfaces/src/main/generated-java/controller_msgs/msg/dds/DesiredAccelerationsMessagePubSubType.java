package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DesiredAccelerationsMessage" defined in "DesiredAccelerationsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DesiredAccelerationsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DesiredAccelerationsMessage_.idl instead.
*
*/
public class DesiredAccelerationsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DesiredAccelerationsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DesiredAccelerationsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3696638deebce87ef704540712dc06d65947710b4a4a7b2910023c1eda36b107";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DesiredAccelerationsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DesiredAccelerationsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DesiredAccelerationsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DesiredAccelerationsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDesiredJointAccelerations().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getDesiredJointAccelerations().size() <= 100)
      cdr.write_type_e(data.getDesiredJointAccelerations());else
          throw new RuntimeException("desired_joint_accelerations field exceeds the maximum length");

      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);
   }

   public static void read(controller_msgs.msg.dds.DesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getDesiredJointAccelerations());	
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DesiredAccelerationsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("desired_joint_accelerations", data.getDesiredJointAccelerations());
      ser.write_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DesiredAccelerationsMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("desired_joint_accelerations", data.getDesiredJointAccelerations());
      ser.read_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

   }

   public static void staticCopy(controller_msgs.msg.dds.DesiredAccelerationsMessage src, controller_msgs.msg.dds.DesiredAccelerationsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.DesiredAccelerationsMessage createData()
   {
      return new controller_msgs.msg.dds.DesiredAccelerationsMessage();
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
   
   public void serialize(controller_msgs.msg.dds.DesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.DesiredAccelerationsMessage src, controller_msgs.msg.dds.DesiredAccelerationsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DesiredAccelerationsMessagePubSubType newInstance()
   {
      return new DesiredAccelerationsMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "JointspaceStreamingMessage" defined in "JointspaceStreamingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from JointspaceStreamingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit JointspaceStreamingMessage_.idl instead.
*
*/
public class JointspaceStreamingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.JointspaceStreamingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::JointspaceStreamingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2d854d26a98d413d12e4876e4a3c49d12568bfde430ba156116fc7613011bcb3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.JointspaceStreamingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.JointspaceStreamingMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (12 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (12 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (12 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointspaceStreamingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointspaceStreamingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getPositions().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getVelocities().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getAccelerations().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.JointspaceStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPositions().size() <= 12)
      cdr.write_type_e(data.getPositions());else
          throw new RuntimeException("positions field exceeds the maximum length");

      if(data.getVelocities().size() <= 12)
      cdr.write_type_e(data.getVelocities());else
          throw new RuntimeException("velocities field exceeds the maximum length");

      if(data.getAccelerations().size() <= 12)
      cdr.write_type_e(data.getAccelerations());else
          throw new RuntimeException("accelerations field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.JointspaceStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPositions());	
      cdr.read_type_e(data.getVelocities());	
      cdr.read_type_e(data.getAccelerations());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.JointspaceStreamingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("positions", data.getPositions());
      ser.write_type_e("velocities", data.getVelocities());
      ser.write_type_e("accelerations", data.getAccelerations());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.JointspaceStreamingMessage data)
   {
      ser.read_type_e("positions", data.getPositions());
      ser.read_type_e("velocities", data.getVelocities());
      ser.read_type_e("accelerations", data.getAccelerations());
   }

   public static void staticCopy(controller_msgs.msg.dds.JointspaceStreamingMessage src, controller_msgs.msg.dds.JointspaceStreamingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.JointspaceStreamingMessage createData()
   {
      return new controller_msgs.msg.dds.JointspaceStreamingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.JointspaceStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.JointspaceStreamingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.JointspaceStreamingMessage src, controller_msgs.msg.dds.JointspaceStreamingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JointspaceStreamingMessagePubSubType newInstance()
   {
      return new JointspaceStreamingMessagePubSubType();
   }
}

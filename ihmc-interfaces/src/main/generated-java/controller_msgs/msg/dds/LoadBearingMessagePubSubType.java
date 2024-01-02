package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LoadBearingMessage" defined in "LoadBearingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LoadBearingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LoadBearingMessage_.idl instead.
*
*/
public class LoadBearingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.LoadBearingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::LoadBearingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "78e9a7a9d15c76fe315fca00b20c1423a40209271ac04985d935c7677afa9948";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.LoadBearingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.LoadBearingMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LoadBearingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.LoadBearingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getContactPoseInBodyFrame(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.LoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getLoad());

      cdr.write_type_6(data.getCoefficientOfFriction());

      geometry_msgs.msg.dds.PosePubSubType.write(data.getContactPoseInBodyFrame(), cdr);
   }

   public static void read(controller_msgs.msg.dds.LoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setLoad(cdr.read_type_7());
      	
      data.setCoefficientOfFriction(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getContactPoseInBodyFrame(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.LoadBearingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("load", data.getLoad());
      ser.write_type_6("coefficient_of_friction", data.getCoefficientOfFriction());
      ser.write_type_a("contact_pose_in_body_frame", new geometry_msgs.msg.dds.PosePubSubType(), data.getContactPoseInBodyFrame());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.LoadBearingMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setLoad(ser.read_type_7("load"));
      data.setCoefficientOfFriction(ser.read_type_6("coefficient_of_friction"));
      ser.read_type_a("contact_pose_in_body_frame", new geometry_msgs.msg.dds.PosePubSubType(), data.getContactPoseInBodyFrame());

   }

   public static void staticCopy(controller_msgs.msg.dds.LoadBearingMessage src, controller_msgs.msg.dds.LoadBearingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.LoadBearingMessage createData()
   {
      return new controller_msgs.msg.dds.LoadBearingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.LoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.LoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.LoadBearingMessage src, controller_msgs.msg.dds.LoadBearingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LoadBearingMessagePubSubType newInstance()
   {
      return new LoadBearingMessagePubSubType();
   }
}

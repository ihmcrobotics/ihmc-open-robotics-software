package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxContactStateMessage" defined in "KinematicsToolboxContactStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxContactStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxContactStateMessage_.idl instead.
*
*/
public class KinematicsToolboxContactStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsToolboxContactStateMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsToolboxContactStateMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 20; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (20 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getContactPointsInBodyFrame().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getContactPointsInBodyFrame().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getContactingBodyIds().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getCenterOfMassMargin());

      if(data.getContactPointsInBodyFrame().size() <= 20)
      cdr.write_type_e(data.getContactPointsInBodyFrame());else
          throw new RuntimeException("contact_points_in_body_frame field exceeds the maximum length");

      if(data.getContactingBodyIds().size() <= 20)
      cdr.write_type_e(data.getContactingBodyIds());else
          throw new RuntimeException("contacting_body_ids field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCenterOfMassMargin(cdr.read_type_6());
      	
      cdr.read_type_e(data.getContactPointsInBodyFrame());	
      cdr.read_type_e(data.getContactingBodyIds());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("center_of_mass_margin", data.getCenterOfMassMargin());
      ser.write_type_e("contact_points_in_body_frame", data.getContactPointsInBodyFrame());
      ser.write_type_e("contacting_body_ids", data.getContactingBodyIds());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCenterOfMassMargin(ser.read_type_6("center_of_mass_margin"));
      ser.read_type_e("contact_points_in_body_frame", data.getContactPointsInBodyFrame());
      ser.read_type_e("contacting_body_ids", data.getContactingBodyIds());
   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage src, controller_msgs.msg.dds.KinematicsToolboxContactStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsToolboxContactStateMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsToolboxContactStateMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsToolboxContactStateMessage src, controller_msgs.msg.dds.KinematicsToolboxContactStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxContactStateMessagePubSubType newInstance()
   {
      return new KinematicsToolboxContactStateMessagePubSubType();
   }
}

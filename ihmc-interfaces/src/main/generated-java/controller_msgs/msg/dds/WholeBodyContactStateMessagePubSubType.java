package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyContactStateMessage" defined in "WholeBodyContactStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyContactStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyContactStateMessage_.idl instead.
*
*/
public class WholeBodyContactStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WholeBodyContactStateMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WholeBodyContactStateMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WholeBodyContactStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WholeBodyContactStateMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyContactStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyContactStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getContactingBodiesHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getContactPointPositions().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getContactPointPositions().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WholeBodyContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getContactingBodiesHashCodes().size() <= 100)
      cdr.write_type_e(data.getContactingBodiesHashCodes());else
          throw new RuntimeException("contacting_bodies_hash_codes field exceeds the maximum length");

      if(data.getContactPointPositions().size() <= 100)
      cdr.write_type_e(data.getContactPointPositions());else
          throw new RuntimeException("contact_point_positions field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.WholeBodyContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getContactingBodiesHashCodes());	
      cdr.read_type_e(data.getContactPointPositions());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WholeBodyContactStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("contacting_bodies_hash_codes", data.getContactingBodiesHashCodes());
      ser.write_type_e("contact_point_positions", data.getContactPointPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WholeBodyContactStateMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("contacting_bodies_hash_codes", data.getContactingBodiesHashCodes());
      ser.read_type_e("contact_point_positions", data.getContactPointPositions());
   }

   public static void staticCopy(controller_msgs.msg.dds.WholeBodyContactStateMessage src, controller_msgs.msg.dds.WholeBodyContactStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WholeBodyContactStateMessage createData()
   {
      return new controller_msgs.msg.dds.WholeBodyContactStateMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WholeBodyContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WholeBodyContactStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WholeBodyContactStateMessage src, controller_msgs.msg.dds.WholeBodyContactStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyContactStateMessagePubSubType newInstance()
   {
      return new WholeBodyContactStateMessagePubSubType();
   }
}

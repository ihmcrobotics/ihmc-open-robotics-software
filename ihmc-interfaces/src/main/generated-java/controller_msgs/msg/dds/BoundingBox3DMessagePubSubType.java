package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BoundingBox3DMessage" defined in "BoundingBox3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BoundingBox3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BoundingBox3DMessage_.idl instead.
*
*/
public class BoundingBox3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BoundingBox3DMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BoundingBox3DMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BoundingBox3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BoundingBox3DMessage data) throws java.io.IOException
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


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BoundingBox3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BoundingBox3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getMinPoint(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getMaxPoint(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BoundingBox3DMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      geometry_msgs.msg.dds.PointPubSubType.write(data.getMinPoint(), cdr);

      geometry_msgs.msg.dds.PointPubSubType.write(data.getMaxPoint(), cdr);
   }

   public static void read(controller_msgs.msg.dds.BoundingBox3DMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getMinPoint(), cdr);	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getMaxPoint(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BoundingBox3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_a("min_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getMinPoint());


      ser.write_type_a("max_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getMaxPoint());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BoundingBox3DMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_a("min_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getMinPoint());


      ser.read_type_a("max_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getMaxPoint());

   }

   public static void staticCopy(controller_msgs.msg.dds.BoundingBox3DMessage src, controller_msgs.msg.dds.BoundingBox3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BoundingBox3DMessage createData()
   {
      return new controller_msgs.msg.dds.BoundingBox3DMessage();
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
   
   public void serialize(controller_msgs.msg.dds.BoundingBox3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BoundingBox3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BoundingBox3DMessage src, controller_msgs.msg.dds.BoundingBox3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BoundingBox3DMessagePubSubType newInstance()
   {
      return new BoundingBox3DMessagePubSubType();
   }
}

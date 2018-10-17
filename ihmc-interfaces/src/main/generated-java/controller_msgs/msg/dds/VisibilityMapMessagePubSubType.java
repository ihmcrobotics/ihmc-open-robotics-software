package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VisibilityMapMessage" defined in "VisibilityMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VisibilityMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VisibilityMapMessage_.idl instead.
*
*/
public class VisibilityMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.VisibilityMapMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::VisibilityMapMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.VisibilityMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.VisibilityMapMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSourcePoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSourcePoints().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTargetPoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getTargetPoints().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.VisibilityMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getMapId());

      if(data.getSourcePoints().size() <= 100)
      cdr.write_type_e(data.getSourcePoints());else
          throw new RuntimeException("source_points field exceeds the maximum length");

      if(data.getTargetPoints().size() <= 100)
      cdr.write_type_e(data.getTargetPoints());else
          throw new RuntimeException("target_points field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.VisibilityMapMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setMapId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getSourcePoints());	
      cdr.read_type_e(data.getTargetPoints());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.VisibilityMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("map_id", data.getMapId());
      ser.write_type_e("source_points", data.getSourcePoints());
      ser.write_type_e("target_points", data.getTargetPoints());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.VisibilityMapMessage data)
   {
      data.setMapId(ser.read_type_4("map_id"));
      ser.read_type_e("source_points", data.getSourcePoints());
      ser.read_type_e("target_points", data.getTargetPoints());
   }

   public static void staticCopy(controller_msgs.msg.dds.VisibilityMapMessage src, controller_msgs.msg.dds.VisibilityMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.VisibilityMapMessage createData()
   {
      return new controller_msgs.msg.dds.VisibilityMapMessage();
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
   
   public void serialize(controller_msgs.msg.dds.VisibilityMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.VisibilityMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.VisibilityMapMessage src, controller_msgs.msg.dds.VisibilityMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VisibilityMapMessagePubSubType newInstance()
   {
      return new VisibilityMapMessagePubSubType();
   }
}

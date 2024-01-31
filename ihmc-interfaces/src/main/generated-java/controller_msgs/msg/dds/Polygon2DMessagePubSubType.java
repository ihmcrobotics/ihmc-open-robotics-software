package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Polygon2DMessage" defined in "Polygon2DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Polygon2DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Polygon2DMessage_.idl instead.
*
*/
public class Polygon2DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.Polygon2DMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::Polygon2DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "baad5603602cdd1f156c6cd14741553878c3617142e82daabc86d4b5b8adc618";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.Polygon2DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.Polygon2DMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.Polygon2DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.Polygon2DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getVertices().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getVertices().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.Polygon2DMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getVertices().size() <= 50)
      cdr.write_type_e(data.getVertices());else
          throw new RuntimeException("vertices field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.Polygon2DMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getVertices());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.Polygon2DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("vertices", data.getVertices());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.Polygon2DMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("vertices", data.getVertices());
   }

   public static void staticCopy(controller_msgs.msg.dds.Polygon2DMessage src, controller_msgs.msg.dds.Polygon2DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.Polygon2DMessage createData()
   {
      return new controller_msgs.msg.dds.Polygon2DMessage();
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
   
   public void serialize(controller_msgs.msg.dds.Polygon2DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.Polygon2DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.Polygon2DMessage src, controller_msgs.msg.dds.Polygon2DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Polygon2DMessagePubSubType newInstance()
   {
      return new Polygon2DMessagePubSubType();
   }
}

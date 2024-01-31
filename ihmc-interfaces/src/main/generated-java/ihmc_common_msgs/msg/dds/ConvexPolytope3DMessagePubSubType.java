package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConvexPolytope3DMessage" defined in "ConvexPolytope3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConvexPolytope3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConvexPolytope3DMessage_.idl instead.
*
*/
public class ConvexPolytope3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::ConvexPolytope3DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ffd5d81d9fdfec94a3be6b6584d05c000e9120941bfce70600683944dd3389b0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getVertices().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getVertices().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getVertices().size() <= 50)
      cdr.write_type_e(data.getVertices());else
          throw new RuntimeException("vertices field exceeds the maximum length");

   }

   public static void read(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getVertices());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("vertices", data.getVertices());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data)
   {
      ser.read_type_e("vertices", data.getVertices());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage src, ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage src, ihmc_common_msgs.msg.dds.ConvexPolytope3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConvexPolytope3DMessagePubSubType newInstance()
   {
      return new ConvexPolytope3DMessagePubSubType();
   }
}

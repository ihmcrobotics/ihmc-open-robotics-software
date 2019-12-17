package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VisibilityClusterPointsMessage" defined in "VisibilityClusterPointsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VisibilityClusterPointsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VisibilityClusterPointsMessage_.idl instead.
*
*/
public class VisibilityClusterPointsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.VisibilityClusterPointsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::VisibilityClusterPointsMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.VisibilityClusterPointsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.VisibilityClusterPointsMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 25; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityClusterPointsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.VisibilityClusterPointsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPoints().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPoints().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.VisibilityClusterPointsMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPoints().size() <= 25)
      cdr.write_type_e(data.getPoints());else
          throw new RuntimeException("points field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.VisibilityClusterPointsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPoints());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.VisibilityClusterPointsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("points", data.getPoints());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.VisibilityClusterPointsMessage data)
   {
      ser.read_type_e("points", data.getPoints());
   }

   public static void staticCopy(controller_msgs.msg.dds.VisibilityClusterPointsMessage src, controller_msgs.msg.dds.VisibilityClusterPointsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.VisibilityClusterPointsMessage createData()
   {
      return new controller_msgs.msg.dds.VisibilityClusterPointsMessage();
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
   
   public void serialize(controller_msgs.msg.dds.VisibilityClusterPointsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.VisibilityClusterPointsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.VisibilityClusterPointsMessage src, controller_msgs.msg.dds.VisibilityClusterPointsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VisibilityClusterPointsMessagePubSubType newInstance()
   {
      return new VisibilityClusterPointsMessagePubSubType();
   }
}

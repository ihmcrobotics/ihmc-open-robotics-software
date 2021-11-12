package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FusedSensorHeadPointCloudMessage" defined in "FusedSensorHeadPointCloudMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FusedSensorHeadPointCloudMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FusedSensorHeadPointCloudMessage_.idl instead.
*
*/
public class FusedSensorHeadPointCloudMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FusedSensorHeadPointCloudMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (355520 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getScan().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSegmentIndex());

      if(data.getScan().size() <= 355520)
      cdr.write_type_e(data.getScan());else
          throw new RuntimeException("scan field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSegmentIndex(cdr.read_type_4());
      	
      cdr.read_type_e(data.getScan());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("segment_index", data.getSegmentIndex());
      ser.write_type_e("scan", data.getScan());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data)
   {
      data.setSegmentIndex(ser.read_type_4("segment_index"));
      ser.read_type_e("scan", data.getScan());
   }

   public static void staticCopy(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage src, controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage createData()
   {
      return new controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage src, controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FusedSensorHeadPointCloudMessagePubSubType newInstance()
   {
      return new FusedSensorHeadPointCloudMessagePubSubType();
   }
}

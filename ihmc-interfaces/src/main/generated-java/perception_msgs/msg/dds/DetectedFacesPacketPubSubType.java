package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DetectedFacesPacket" defined in "DetectedFacesPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DetectedFacesPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DetectedFacesPacket_.idl instead.
*
*/
public class DetectedFacesPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DetectedFacesPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DetectedFacesPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3e9c0bab8af8d6ae5c0362b743269a9ec7e80feb9f1eb7700324c6435bb07e25";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DetectedFacesPacket data) throws java.io.IOException
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
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedFacesPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DetectedFacesPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getIds().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIds().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPositions().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPositions().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getIds().size() <= 100)
      cdr.write_type_e(data.getIds());else
          throw new RuntimeException("ids field exceeds the maximum length");

      if(data.getPositions().size() <= 100)
      cdr.write_type_e(data.getPositions());else
          throw new RuntimeException("positions field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getIds());	
      cdr.read_type_e(data.getPositions());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("ids", data.getIds());
      ser.write_type_e("positions", data.getPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DetectedFacesPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("ids", data.getIds());
      ser.read_type_e("positions", data.getPositions());
   }

   public static void staticCopy(perception_msgs.msg.dds.DetectedFacesPacket src, perception_msgs.msg.dds.DetectedFacesPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DetectedFacesPacket createData()
   {
      return new perception_msgs.msg.dds.DetectedFacesPacket();
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
   
   public void serialize(perception_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DetectedFacesPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DetectedFacesPacket src, perception_msgs.msg.dds.DetectedFacesPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DetectedFacesPacketPubSubType newInstance()
   {
      return new DetectedFacesPacketPubSubType();
   }
}

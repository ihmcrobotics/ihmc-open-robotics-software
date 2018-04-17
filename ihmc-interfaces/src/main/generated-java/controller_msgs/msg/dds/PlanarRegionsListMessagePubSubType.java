package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PlanarRegionsListMessage" defined in "PlanarRegionsListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PlanarRegionsListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PlanarRegionsListMessage_.idl instead.
*
*/
public class PlanarRegionsListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PlanarRegionsListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PlanarRegionsListMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PlanarRegionsListMessage data) throws java.io.IOException
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
          current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarRegionsListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PlanarRegionsListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPlanarRegions().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getPlanarRegions().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getPlanarRegions().size() <= 100)
      cdr.write_type_e(data.getPlanarRegions());else
          throw new RuntimeException("planar_regions field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getPlanarRegions());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("planar_regions", data.getPlanarRegions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PlanarRegionsListMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("planar_regions", data.getPlanarRegions());
   }

   public static void staticCopy(controller_msgs.msg.dds.PlanarRegionsListMessage src, controller_msgs.msg.dds.PlanarRegionsListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PlanarRegionsListMessage createData()
   {
      return new controller_msgs.msg.dds.PlanarRegionsListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PlanarRegionsListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PlanarRegionsListMessage src, controller_msgs.msg.dds.PlanarRegionsListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PlanarRegionsListMessagePubSubType newInstance()
   {
      return new PlanarRegionsListMessagePubSubType();
   }
}

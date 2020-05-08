package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerOccupancyMapMessage" defined in "FootstepPlannerOccupancyMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerOccupancyMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerOccupancyMapMessage_.idl instead.
*
*/
public class FootstepPlannerOccupancyMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlannerOccupancyMapMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10000; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepPlannerCellMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getOccupiedCells().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepPlannerCellMessagePubSubType.getCdrSerializedSize(data.getOccupiedCells().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      if(data.getOccupiedCells().size() <= 10000)
      cdr.write_type_e(data.getOccupiedCells());else
          throw new RuntimeException("occupied_cells field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      cdr.read_type_e(data.getOccupiedCells());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_e("occupied_cells", data.getOccupiedCells());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_e("occupied_cells", data.getOccupiedCells());
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage src, controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage src, controller_msgs.msg.dds.FootstepPlannerOccupancyMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerOccupancyMapMessagePubSubType newInstance()
   {
      return new FootstepPlannerOccupancyMapMessagePubSubType();
   }
}

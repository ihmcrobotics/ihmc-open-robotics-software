package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerCellMessage" defined in "FootstepPlannerCellMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerCellMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerCellMessage_.idl instead.
*
*/
public class FootstepPlannerCellMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlannerCellMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlannerCellMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlannerCellMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlannerCellMessage data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerCellMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerCellMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlannerCellMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_2(data.getXIndex());


      cdr.write_type_2(data.getYIndex());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlannerCellMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setXIndex(cdr.read_type_2());
      	

      data.setYIndex(cdr.read_type_2());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlannerCellMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_2("x_index", data.getXIndex());

      ser.write_type_2("y_index", data.getYIndex());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlannerCellMessage data)
   {

      data.setXIndex(ser.read_type_2("x_index"));

      data.setYIndex(ser.read_type_2("y_index"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlannerCellMessage src, controller_msgs.msg.dds.FootstepPlannerCellMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlannerCellMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepPlannerCellMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlannerCellMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlannerCellMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlannerCellMessage src, controller_msgs.msg.dds.FootstepPlannerCellMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerCellMessagePubSubType newInstance()
   {
      return new FootstepPlannerCellMessagePubSubType();
   }
}

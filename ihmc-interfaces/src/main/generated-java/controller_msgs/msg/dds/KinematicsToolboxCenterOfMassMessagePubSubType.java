package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxCenterOfMassMessage" defined in "KinematicsToolboxCenterOfMassMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxCenterOfMassMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxCenterOfMassMessage_.idl instead.
*
*/
public class KinematicsToolboxCenterOfMassMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsToolboxCenterOfMassMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data) throws java.io.IOException
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


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredPositionInWorld(), current_alignment);


      current_alignment += controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getSelectionMatrix(), current_alignment);


      current_alignment += controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getWeights(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredPositionInWorld(), cdr);

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getSelectionMatrix(), cdr);

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getWeights(), cdr);
   }

   public static void read(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredPositionInWorld(), cdr);	

      controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getSelectionMatrix(), cdr);	

      controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getWeights(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());


      ser.write_type_a("selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());


      ser.write_type_a("weights", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeights());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());


      ser.read_type_a("selection_matrix", new controller_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());


      ser.read_type_a("weights", new controller_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeights());

   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage src, controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage createData()
   {
      return new controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage src, controller_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxCenterOfMassMessagePubSubType newInstance()
   {
      return new KinematicsToolboxCenterOfMassMessagePubSubType();
   }
}

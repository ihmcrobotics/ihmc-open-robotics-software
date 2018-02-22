package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "ElectricJointData" defined in "ElectricJointData_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from ElectricJointData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit ElectricJointData_.idl instead.
 */
public class ElectricJointDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ElectricJointData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ElectricJointData_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public ElectricJointDataPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ElectricJointData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ElectricJointData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ElectricJointData data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_7(data.getJoint_enabled());

      cdr.write_type_5(data.getJoint_temperature());

      cdr.write_type_5(data.getJoint_current());
   }

   public static void read(controller_msgs.msg.dds.ElectricJointData data, us.ihmc.idl.CDR cdr)
   {

      data.setJoint_enabled(cdr.read_type_7());

      data.setJoint_temperature(cdr.read_type_5());

      data.setJoint_current(cdr.read_type_5());
   }

   public static void staticCopy(controller_msgs.msg.dds.ElectricJointData src, controller_msgs.msg.dds.ElectricJointData dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.ElectricJointData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ElectricJointData data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ElectricJointData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("joint_enabled", data.getJoint_enabled());

      ser.write_type_5("joint_temperature", data.getJoint_temperature());

      ser.write_type_5("joint_current", data.getJoint_current());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ElectricJointData data)
   {
      data.setJoint_enabled(ser.read_type_7("joint_enabled"));

      data.setJoint_temperature(ser.read_type_5("joint_temperature"));

      data.setJoint_current(ser.read_type_5("joint_current"));
   }

   @Override
   public controller_msgs.msg.dds.ElectricJointData createData()
   {
      return new controller_msgs.msg.dds.ElectricJointData();
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

   public void serialize(controller_msgs.msg.dds.ElectricJointData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ElectricJointData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.ElectricJointData src, controller_msgs.msg.dds.ElectricJointData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ElectricJointDataPubSubType newInstance()
   {
      return new ElectricJointDataPubSubType();
   }
}
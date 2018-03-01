package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "AtlasAuxiliaryData" defined in "AtlasAuxiliaryData_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from AtlasAuxiliaryData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit AtlasAuxiliaryData_.idl instead.
 */
public class AtlasAuxiliaryDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.AtlasAuxiliaryData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::AtlasAuxiliaryData_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public AtlasAuxiliaryDataPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);
      for (int a = 0; a < (6); ++a)
      {
         current_alignment += controller_msgs.msg.dds.ElectricJointDataPubSubType.getMaxCdrSerializedSize(current_alignment);
      }
      for (int a = 0; a < (15); ++a)
      {
         current_alignment += controller_msgs.msg.dds.RawImuDataPubSubType.getMaxCdrSerializedSize(current_alignment);
      }
      current_alignment += controller_msgs.msg.dds.BatteryStatePubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += controller_msgs.msg.dds.PumpStatePubSubType.getMaxCdrSerializedSize(current_alignment);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AtlasAuxiliaryData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AtlasAuxiliaryData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);
      for (int a = 0; a < data.getElectric_joint_data().length; ++a)
      {
         current_alignment += controller_msgs.msg.dds.ElectricJointDataPubSubType.getCdrSerializedSize(data.getElectric_joint_data()[a], current_alignment);
      }
      for (int a = 0; a < data.getRaw_imu_data().length; ++a)
      {
         current_alignment += controller_msgs.msg.dds.RawImuDataPubSubType.getCdrSerializedSize(data.getRaw_imu_data()[a], current_alignment);
      }
      current_alignment += controller_msgs.msg.dds.BatteryStatePubSubType.getCdrSerializedSize(data.getBattery_state(), current_alignment);
      current_alignment += controller_msgs.msg.dds.PumpStatePubSubType.getCdrSerializedSize(data.getPump_state(), current_alignment);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.AtlasAuxiliaryData data, us.ihmc.idl.CDR cdr)
   {

      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);

      for (int a = 0; a < data.getElectric_joint_data().length; ++a)
      {
         controller_msgs.msg.dds.ElectricJointDataPubSubType.write(data.getElectric_joint_data()[a], cdr);
      }

      for (int a = 0; a < data.getRaw_imu_data().length; ++a)
      {
         controller_msgs.msg.dds.RawImuDataPubSubType.write(data.getRaw_imu_data()[a], cdr);
      }

      controller_msgs.msg.dds.BatteryStatePubSubType.write(data.getBattery_state(), cdr);

      controller_msgs.msg.dds.PumpStatePubSubType.write(data.getPump_state(), cdr);
   }

   public static void read(controller_msgs.msg.dds.AtlasAuxiliaryData data, us.ihmc.idl.CDR cdr)
   {

      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);

      for (int a = 0; a < data.getElectric_joint_data().length; ++a)
      {
         controller_msgs.msg.dds.ElectricJointDataPubSubType.read(data.getElectric_joint_data()[a], cdr);
      }

      for (int a = 0; a < data.getRaw_imu_data().length; ++a)
      {
         controller_msgs.msg.dds.RawImuDataPubSubType.read(data.getRaw_imu_data()[a], cdr);
      }

      controller_msgs.msg.dds.BatteryStatePubSubType.read(data.getBattery_state(), cdr);

      controller_msgs.msg.dds.PumpStatePubSubType.read(data.getPump_state(), cdr);
   }

   public static void staticCopy(controller_msgs.msg.dds.AtlasAuxiliaryData src, controller_msgs.msg.dds.AtlasAuxiliaryData dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.AtlasAuxiliaryData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.AtlasAuxiliaryData data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.AtlasAuxiliaryData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_f("electric_joint_data", new controller_msgs.msg.dds.ElectricJointDataPubSubType(), data.getElectric_joint_data());
      ser.write_type_f("raw_imu_data", new controller_msgs.msg.dds.RawImuDataPubSubType(), data.getRaw_imu_data());
      ser.write_type_a("battery_state", new controller_msgs.msg.dds.BatteryStatePubSubType(), data.getBattery_state());

      ser.write_type_a("pump_state", new controller_msgs.msg.dds.PumpStatePubSubType(), data.getPump_state());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.AtlasAuxiliaryData data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_f("electric_joint_data", new controller_msgs.msg.dds.ElectricJointDataPubSubType(), data.getElectric_joint_data());

      ser.read_type_f("raw_imu_data", new controller_msgs.msg.dds.RawImuDataPubSubType(), data.getRaw_imu_data());

      ser.read_type_a("battery_state", new controller_msgs.msg.dds.BatteryStatePubSubType(), data.getBattery_state());

      ser.read_type_a("pump_state", new controller_msgs.msg.dds.PumpStatePubSubType(), data.getPump_state());
   }

   @Override
   public controller_msgs.msg.dds.AtlasAuxiliaryData createData()
   {
      return new controller_msgs.msg.dds.AtlasAuxiliaryData();
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

   public void serialize(controller_msgs.msg.dds.AtlasAuxiliaryData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.AtlasAuxiliaryData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.AtlasAuxiliaryData src, controller_msgs.msg.dds.AtlasAuxiliaryData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AtlasAuxiliaryDataPubSubType newInstance()
   {
      return new AtlasAuxiliaryDataPubSubType();
   }
}
package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "BatteryState" defined in "BatteryState_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from BatteryState_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit BatteryState_.idl instead.
 */
public class BatteryStatePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BatteryState>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BatteryState_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public BatteryStatePubSubType()
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BatteryState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BatteryState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BatteryState data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_7(data.getBatteryCharging());

      cdr.write_type_5(data.getBatteryVoltage());

      cdr.write_type_5(data.getBatteryCurrent());

      cdr.write_type_5(data.getRemainingBatteryTime());

      cdr.write_type_5(data.getRemainingAmpHours());

      cdr.write_type_5(data.getRemainingChargePercentage());

      cdr.write_type_11(data.getBatteryCycleCount());
   }

   public static void read(controller_msgs.msg.dds.BatteryState data, us.ihmc.idl.CDR cdr)
   {

      data.setBatteryCharging(cdr.read_type_7());

      data.setBatteryVoltage(cdr.read_type_5());

      data.setBatteryCurrent(cdr.read_type_5());

      data.setRemainingBatteryTime(cdr.read_type_5());

      data.setRemainingAmpHours(cdr.read_type_5());

      data.setRemainingChargePercentage(cdr.read_type_5());

      data.setBatteryCycleCount(cdr.read_type_11());
   }

   public static void staticCopy(controller_msgs.msg.dds.BatteryState src, controller_msgs.msg.dds.BatteryState dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.BatteryState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BatteryState data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BatteryState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("battery_charging", data.getBatteryCharging());

      ser.write_type_5("battery_voltage", data.getBatteryVoltage());

      ser.write_type_5("battery_current", data.getBatteryCurrent());

      ser.write_type_5("remaining_battery_time", data.getRemainingBatteryTime());

      ser.write_type_5("remaining_amp_hours", data.getRemainingAmpHours());

      ser.write_type_5("remaining_charge_percentage", data.getRemainingChargePercentage());

      ser.write_type_11("battery_cycle_count", data.getBatteryCycleCount());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BatteryState data)
   {
      data.setBatteryCharging(ser.read_type_7("battery_charging"));

      data.setBatteryVoltage(ser.read_type_5("battery_voltage"));

      data.setBatteryCurrent(ser.read_type_5("battery_current"));

      data.setRemainingBatteryTime(ser.read_type_5("remaining_battery_time"));

      data.setRemainingAmpHours(ser.read_type_5("remaining_amp_hours"));

      data.setRemainingChargePercentage(ser.read_type_5("remaining_charge_percentage"));

      data.setBatteryCycleCount(ser.read_type_11("battery_cycle_count"));
   }

   @Override
   public controller_msgs.msg.dds.BatteryState createData()
   {
      return new controller_msgs.msg.dds.BatteryState();
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

   public void serialize(controller_msgs.msg.dds.BatteryState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BatteryState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.BatteryState src, controller_msgs.msg.dds.BatteryState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BatteryStatePubSubType newInstance()
   {
      return new BatteryStatePubSubType();
   }
}
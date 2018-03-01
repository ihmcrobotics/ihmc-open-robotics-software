package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "PumpState" defined in "PumpState_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from PumpState_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit PumpState_.idl instead.
 */
public class PumpStatePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PumpState>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PumpState_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public PumpStatePubSubType()
   {

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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PumpState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PumpState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PumpState data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_5(data.getPump_inlet_pressure());

      cdr.write_type_5(data.getPump_supply_pressure());

      cdr.write_type_5(data.getAir_sump_pressure());

      cdr.write_type_5(data.getPump_supply_temperature());

      cdr.write_type_5(data.getPump_rpm());

      cdr.write_type_5(data.getMotor_temperature());

      cdr.write_type_5(data.getMotor_driver_temperature());
   }

   public static void read(controller_msgs.msg.dds.PumpState data, us.ihmc.idl.CDR cdr)
   {

      data.setPump_inlet_pressure(cdr.read_type_5());

      data.setPump_supply_pressure(cdr.read_type_5());

      data.setAir_sump_pressure(cdr.read_type_5());

      data.setPump_supply_temperature(cdr.read_type_5());

      data.setPump_rpm(cdr.read_type_5());

      data.setMotor_temperature(cdr.read_type_5());

      data.setMotor_driver_temperature(cdr.read_type_5());
   }

   public static void staticCopy(controller_msgs.msg.dds.PumpState src, controller_msgs.msg.dds.PumpState dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.PumpState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PumpState data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PumpState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_5("pump_inlet_pressure", data.getPump_inlet_pressure());

      ser.write_type_5("pump_supply_pressure", data.getPump_supply_pressure());

      ser.write_type_5("air_sump_pressure", data.getAir_sump_pressure());

      ser.write_type_5("pump_supply_temperature", data.getPump_supply_temperature());

      ser.write_type_5("pump_rpm", data.getPump_rpm());

      ser.write_type_5("motor_temperature", data.getMotor_temperature());

      ser.write_type_5("motor_driver_temperature", data.getMotor_driver_temperature());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PumpState data)
   {
      data.setPump_inlet_pressure(ser.read_type_5("pump_inlet_pressure"));

      data.setPump_supply_pressure(ser.read_type_5("pump_supply_pressure"));

      data.setAir_sump_pressure(ser.read_type_5("air_sump_pressure"));

      data.setPump_supply_temperature(ser.read_type_5("pump_supply_temperature"));

      data.setPump_rpm(ser.read_type_5("pump_rpm"));

      data.setMotor_temperature(ser.read_type_5("motor_temperature"));

      data.setMotor_driver_temperature(ser.read_type_5("motor_driver_temperature"));
   }

   @Override
   public controller_msgs.msg.dds.PumpState createData()
   {
      return new controller_msgs.msg.dds.PumpState();
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

   public void serialize(controller_msgs.msg.dds.PumpState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PumpState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.PumpState src, controller_msgs.msg.dds.PumpState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PumpStatePubSubType newInstance()
   {
      return new PumpStatePubSubType();
   }
}
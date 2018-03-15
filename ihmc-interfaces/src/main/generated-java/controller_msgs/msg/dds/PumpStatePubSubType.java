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

      cdr.write_type_5(data.getPumpInletPressure());

      cdr.write_type_5(data.getPumpSupplyPressure());

      cdr.write_type_5(data.getAirSumpPressure());

      cdr.write_type_5(data.getPumpSupplyTemperature());

      cdr.write_type_5(data.getPumpRpm());

      cdr.write_type_5(data.getMotorTemperature());

      cdr.write_type_5(data.getMotorDriverTemperature());
   }

   public static void read(controller_msgs.msg.dds.PumpState data, us.ihmc.idl.CDR cdr)
   {

      data.setPumpInletPressure(cdr.read_type_5());

      data.setPumpSupplyPressure(cdr.read_type_5());

      data.setAirSumpPressure(cdr.read_type_5());

      data.setPumpSupplyTemperature(cdr.read_type_5());

      data.setPumpRpm(cdr.read_type_5());

      data.setMotorTemperature(cdr.read_type_5());

      data.setMotorDriverTemperature(cdr.read_type_5());
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
      ser.write_type_5("pump_inlet_pressure", data.getPumpInletPressure());

      ser.write_type_5("pump_supply_pressure", data.getPumpSupplyPressure());

      ser.write_type_5("air_sump_pressure", data.getAirSumpPressure());

      ser.write_type_5("pump_supply_temperature", data.getPumpSupplyTemperature());

      ser.write_type_5("pump_rpm", data.getPumpRpm());

      ser.write_type_5("motor_temperature", data.getMotorTemperature());

      ser.write_type_5("motor_driver_temperature", data.getMotorDriverTemperature());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PumpState data)
   {
      data.setPumpInletPressure(ser.read_type_5("pump_inlet_pressure"));

      data.setPumpSupplyPressure(ser.read_type_5("pump_supply_pressure"));

      data.setAirSumpPressure(ser.read_type_5("air_sump_pressure"));

      data.setPumpSupplyTemperature(ser.read_type_5("pump_supply_temperature"));

      data.setPumpRpm(ser.read_type_5("pump_rpm"));

      data.setMotorTemperature(ser.read_type_5("motor_temperature"));

      data.setMotorDriverTemperature(ser.read_type_5("motor_driver_temperature"));
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
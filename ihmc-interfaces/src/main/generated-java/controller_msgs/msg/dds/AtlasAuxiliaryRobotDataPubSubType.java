package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AtlasAuxiliaryRobotData" defined in "AtlasAuxiliaryRobotData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AtlasAuxiliaryRobotData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AtlasAuxiliaryRobotData_.idl instead.
*
*/
public class AtlasAuxiliaryRobotDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.AtlasAuxiliaryRobotData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::AtlasAuxiliaryRobotData_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.AtlasAuxiliaryRobotData data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (6 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (6 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (6 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((15) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((15) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      for(int i0 = 0; i0 < (15); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}

      for(int i0 = 0; i0 < (15); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getElectricJointTemperatures().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getElectricJointCurrents().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getElectricJointEnabledArray().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += ((15) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((15) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      for(int i0 = 0; i0 < data.getRawImuRates().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRawImuRates()[i0], current_alignment);
      }

      for(int i0 = 0; i0 < data.getRawImuDeltas().length; ++i0)
      {
              current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRawImuDeltas()[i0], current_alignment);
      }

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      if(data.getElectricJointTemperatures().size() <= 6)
      cdr.write_type_e(data.getElectricJointTemperatures());else
          throw new RuntimeException("electric_joint_temperatures field exceeds the maximum length");


      if(data.getElectricJointCurrents().size() <= 6)
      cdr.write_type_e(data.getElectricJointCurrents());else
          throw new RuntimeException("electric_joint_currents field exceeds the maximum length");


      if(data.getElectricJointEnabledArray().size() <= 6)
      cdr.write_type_e(data.getElectricJointEnabledArray());else
          throw new RuntimeException("electric_joint_enabled_array field exceeds the maximum length");


      for(int i0 = 0; i0 < data.getRawImuTimestamps().length; ++i0)
      {
        	cdr.write_type_5(data.getRawImuTimestamps()[i0]);	
      }


      for(int i0 = 0; i0 < data.getRawImuPacketCounts().length; ++i0)
      {
        	cdr.write_type_5(data.getRawImuPacketCounts()[i0]);	
      }


      for(int i0 = 0; i0 < data.getRawImuRates().length; ++i0)
      {
        	geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRawImuRates()[i0], cdr);		
      }


      for(int i0 = 0; i0 < data.getRawImuDeltas().length; ++i0)
      {
        	geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRawImuDeltas()[i0], cdr);		
      }


      cdr.write_type_7(data.getBatteryCharging());


      cdr.write_type_5(data.getBatteryVoltage());


      cdr.write_type_5(data.getBatteryCurrent());


      cdr.write_type_5(data.getRemainingBatteryTime());


      cdr.write_type_5(data.getRemainingAmpHours());


      cdr.write_type_5(data.getRemainingChargePercentage());


      cdr.write_type_11(data.getBatteryCycleCount());


      cdr.write_type_5(data.getPumpInletPressure());


      cdr.write_type_5(data.getPumpSupplyPressure());


      cdr.write_type_5(data.getAirSumpPressure());


      cdr.write_type_5(data.getPumpSupplyTemperature());


      cdr.write_type_5(data.getPumpRpm());


      cdr.write_type_5(data.getMotorTemperature());


      cdr.write_type_5(data.getMotorDriverTemperature());

   }

   public static void read(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      cdr.read_type_e(data.getElectricJointTemperatures());	

      cdr.read_type_e(data.getElectricJointCurrents());	

      cdr.read_type_e(data.getElectricJointEnabledArray());	

      for(int i0 = 0; i0 < data.getRawImuTimestamps().length; ++i0)
      {
        	data.getRawImuTimestamps()[i0] = cdr.read_type_5();
        	
      }
      	

      for(int i0 = 0; i0 < data.getRawImuPacketCounts().length; ++i0)
      {
        	data.getRawImuPacketCounts()[i0] = cdr.read_type_5();
        	
      }
      	

      for(int i0 = 0; i0 < data.getRawImuRates().length; ++i0)
      {
        	geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRawImuRates()[i0], cdr);	
      }
      	

      for(int i0 = 0; i0 < data.getRawImuDeltas().length; ++i0)
      {
        	geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRawImuDeltas()[i0], cdr);	
      }
      	

      data.setBatteryCharging(cdr.read_type_7());
      	

      data.setBatteryVoltage(cdr.read_type_5());
      	

      data.setBatteryCurrent(cdr.read_type_5());
      	

      data.setRemainingBatteryTime(cdr.read_type_5());
      	

      data.setRemainingAmpHours(cdr.read_type_5());
      	

      data.setRemainingChargePercentage(cdr.read_type_5());
      	

      data.setBatteryCycleCount(cdr.read_type_11());
      	

      data.setPumpInletPressure(cdr.read_type_5());
      	

      data.setPumpSupplyPressure(cdr.read_type_5());
      	

      data.setAirSumpPressure(cdr.read_type_5());
      	

      data.setPumpSupplyTemperature(cdr.read_type_5());
      	

      data.setPumpRpm(cdr.read_type_5());
      	

      data.setMotorTemperature(cdr.read_type_5());
      	

      data.setMotorDriverTemperature(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_e("electric_joint_temperatures", data.getElectricJointTemperatures());

      ser.write_type_e("electric_joint_currents", data.getElectricJointCurrents());

      ser.write_type_e("electric_joint_enabled_array", data.getElectricJointEnabledArray());

      ser.write_type_f("raw_imu_timestamps", data.getRawImuTimestamps());

      ser.write_type_f("raw_imu_packet_counts", data.getRawImuPacketCounts());

      ser.write_type_f("raw_imu_rates", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRawImuRates());

      ser.write_type_f("raw_imu_deltas", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRawImuDeltas());

      ser.write_type_7("battery_charging", data.getBatteryCharging());

      ser.write_type_5("battery_voltage", data.getBatteryVoltage());

      ser.write_type_5("battery_current", data.getBatteryCurrent());

      ser.write_type_5("remaining_battery_time", data.getRemainingBatteryTime());

      ser.write_type_5("remaining_amp_hours", data.getRemainingAmpHours());

      ser.write_type_5("remaining_charge_percentage", data.getRemainingChargePercentage());

      ser.write_type_11("battery_cycle_count", data.getBatteryCycleCount());

      ser.write_type_5("pump_inlet_pressure", data.getPumpInletPressure());

      ser.write_type_5("pump_supply_pressure", data.getPumpSupplyPressure());

      ser.write_type_5("air_sump_pressure", data.getAirSumpPressure());

      ser.write_type_5("pump_supply_temperature", data.getPumpSupplyTemperature());

      ser.write_type_5("pump_rpm", data.getPumpRpm());

      ser.write_type_5("motor_temperature", data.getMotorTemperature());

      ser.write_type_5("motor_driver_temperature", data.getMotorDriverTemperature());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.AtlasAuxiliaryRobotData data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_e("electric_joint_temperatures", data.getElectricJointTemperatures());

      ser.read_type_e("electric_joint_currents", data.getElectricJointCurrents());

      ser.read_type_e("electric_joint_enabled_array", data.getElectricJointEnabledArray());

      ser.read_type_f("raw_imu_timestamps", data.getRawImuTimestamps());

      ser.read_type_f("raw_imu_packet_counts", data.getRawImuPacketCounts());

      ser.read_type_f("raw_imu_rates", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRawImuRates());

      ser.read_type_f("raw_imu_deltas", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRawImuDeltas());

      data.setBatteryCharging(ser.read_type_7("battery_charging"));

      data.setBatteryVoltage(ser.read_type_5("battery_voltage"));

      data.setBatteryCurrent(ser.read_type_5("battery_current"));

      data.setRemainingBatteryTime(ser.read_type_5("remaining_battery_time"));

      data.setRemainingAmpHours(ser.read_type_5("remaining_amp_hours"));

      data.setRemainingChargePercentage(ser.read_type_5("remaining_charge_percentage"));

      data.setBatteryCycleCount(ser.read_type_11("battery_cycle_count"));

      data.setPumpInletPressure(ser.read_type_5("pump_inlet_pressure"));

      data.setPumpSupplyPressure(ser.read_type_5("pump_supply_pressure"));

      data.setAirSumpPressure(ser.read_type_5("air_sump_pressure"));

      data.setPumpSupplyTemperature(ser.read_type_5("pump_supply_temperature"));

      data.setPumpRpm(ser.read_type_5("pump_rpm"));

      data.setMotorTemperature(ser.read_type_5("motor_temperature"));

      data.setMotorDriverTemperature(ser.read_type_5("motor_driver_temperature"));
   }

   public static void staticCopy(controller_msgs.msg.dds.AtlasAuxiliaryRobotData src, controller_msgs.msg.dds.AtlasAuxiliaryRobotData dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.AtlasAuxiliaryRobotData createData()
   {
      return new controller_msgs.msg.dds.AtlasAuxiliaryRobotData();
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
   
   public void serialize(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.AtlasAuxiliaryRobotData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.AtlasAuxiliaryRobotData src, controller_msgs.msg.dds.AtlasAuxiliaryRobotData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AtlasAuxiliaryRobotDataPubSubType newInstance()
   {
      return new AtlasAuxiliaryRobotDataPubSubType();
   }
}

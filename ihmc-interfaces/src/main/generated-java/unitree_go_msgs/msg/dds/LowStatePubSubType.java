package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LowState" defined in "LowState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LowState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LowState_.idl instead.
*
*/
public class LowStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.LowState>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::LowState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "53485f2f1edd3f7857e6d37e57e1936c048f61b66249412900ea033946118f88";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.LowState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.LowState data) throws java.io.IOException
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

      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += unitree_go_msgs.msg.dds.IMUStatePubSubType.getMaxCdrSerializedSize(current_alignment);

      for(int i0 = 0; i0 < (20); ++i0)
      {
          current_alignment += unitree_go_msgs.msg.dds.MotorStatePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += unitree_go_msgs.msg.dds.BmsStatePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ((4) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += ((4) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((40) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.LowState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.LowState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += unitree_go_msgs.msg.dds.IMUStatePubSubType.getCdrSerializedSize(data.getImuState(), current_alignment);

      for(int i0 = 0; i0 < data.getMotorState().length; ++i0)
      {
              current_alignment += unitree_go_msgs.msg.dds.MotorStatePubSubType.getCdrSerializedSize(data.getMotorState()[i0], current_alignment);
      }
      current_alignment += unitree_go_msgs.msg.dds.BmsStatePubSubType.getCdrSerializedSize(data.getBmsState(), current_alignment);

      current_alignment += ((4) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += ((4) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((40) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getHead().length; ++i0)
      {
        	cdr.write_type_9(data.getHead()[i0]);	
      }

      cdr.write_type_9(data.getLevelFlag());

      cdr.write_type_9(data.getFrameReserve());

      cdr.write_type_3(data.getBandwidth());

      unitree_go_msgs.msg.dds.IMUStatePubSubType.write(data.getImuState(), cdr);
      for(int i0 = 0; i0 < data.getMotorState().length; ++i0)
      {
        	unitree_go_msgs.msg.dds.MotorStatePubSubType.write(data.getMotorState()[i0], cdr);		
      }

      unitree_go_msgs.msg.dds.BmsStatePubSubType.write(data.getBmsState(), cdr);
      for(int i0 = 0; i0 < data.getFootForce().length; ++i0)
      {
        	cdr.write_type_1(data.getFootForce()[i0]);	
      }

      for(int i0 = 0; i0 < data.getFootForceEst().length; ++i0)
      {
        	cdr.write_type_1(data.getFootForceEst()[i0]);	
      }

      cdr.write_type_4(data.getTick());

      for(int i0 = 0; i0 < data.getWirelessRemote().length; ++i0)
      {
        	cdr.write_type_9(data.getWirelessRemote()[i0]);	
      }

      cdr.write_type_9(data.getBitFlag());

      cdr.write_type_5(data.getAdcReel());

      cdr.write_type_9(data.getTemperatureNtc1());

      cdr.write_type_9(data.getTemperatureNtc2());

      cdr.write_type_5(data.getPowerV());

      cdr.write_type_5(data.getPowerA());

      cdr.write_type_4(data.getReserve());

      cdr.write_type_4(data.getCrc());

   }

   public static void read(unitree_go_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getHead().length; ++i0)
      {
        	data.getHead()[i0] = cdr.read_type_9();
        	
      }
      	
      data.setLevelFlag(cdr.read_type_9());
      	
      data.setFrameReserve(cdr.read_type_9());
      	
      data.setBandwidth(cdr.read_type_3());
      	
      unitree_go_msgs.msg.dds.IMUStatePubSubType.read(data.getImuState(), cdr);	
      for(int i0 = 0; i0 < data.getMotorState().length; ++i0)
      {
        	unitree_go_msgs.msg.dds.MotorStatePubSubType.read(data.getMotorState()[i0], cdr);	
      }
      	
      unitree_go_msgs.msg.dds.BmsStatePubSubType.read(data.getBmsState(), cdr);	
      for(int i0 = 0; i0 < data.getFootForce().length; ++i0)
      {
        	data.getFootForce()[i0] = cdr.read_type_1();
        	
      }
      	
      for(int i0 = 0; i0 < data.getFootForceEst().length; ++i0)
      {
        	data.getFootForceEst()[i0] = cdr.read_type_1();
        	
      }
      	
      data.setTick(cdr.read_type_4());
      	
      for(int i0 = 0; i0 < data.getWirelessRemote().length; ++i0)
      {
        	data.getWirelessRemote()[i0] = cdr.read_type_9();
        	
      }
      	
      data.setBitFlag(cdr.read_type_9());
      	
      data.setAdcReel(cdr.read_type_5());
      	
      data.setTemperatureNtc1(cdr.read_type_9());
      	
      data.setTemperatureNtc2(cdr.read_type_9());
      	
      data.setPowerV(cdr.read_type_5());
      	
      data.setPowerA(cdr.read_type_5());
      	
      data.setReserve(cdr.read_type_4());
      	
      data.setCrc(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.LowState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("head", data.getHead());
      ser.write_type_9("level_flag", data.getLevelFlag());
      ser.write_type_9("frame_reserve", data.getFrameReserve());
      ser.write_type_3("bandwidth", data.getBandwidth());
      ser.write_type_a("imu_state", new unitree_go_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      ser.write_type_f("motor_state", new unitree_go_msgs.msg.dds.MotorStatePubSubType(), data.getMotorState());
      ser.write_type_a("bms_state", new unitree_go_msgs.msg.dds.BmsStatePubSubType(), data.getBmsState());

      ser.write_type_f("foot_force", data.getFootForce());
      ser.write_type_f("foot_force_est", data.getFootForceEst());
      ser.write_type_4("tick", data.getTick());
      ser.write_type_f("wireless_remote", data.getWirelessRemote());
      ser.write_type_9("bit_flag", data.getBitFlag());
      ser.write_type_5("adc_reel", data.getAdcReel());
      ser.write_type_9("temperature_ntc1", data.getTemperatureNtc1());
      ser.write_type_9("temperature_ntc2", data.getTemperatureNtc2());
      ser.write_type_5("power_v", data.getPowerV());
      ser.write_type_5("power_a", data.getPowerA());
      ser.write_type_4("reserve", data.getReserve());
      ser.write_type_4("crc", data.getCrc());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.LowState data)
   {
      ser.read_type_f("head", data.getHead());
      data.setLevelFlag(ser.read_type_9("level_flag"));
      data.setFrameReserve(ser.read_type_9("frame_reserve"));
      data.setBandwidth(ser.read_type_3("bandwidth"));
      ser.read_type_a("imu_state", new unitree_go_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      ser.read_type_f("motor_state", new unitree_go_msgs.msg.dds.MotorStatePubSubType(), data.getMotorState());
      ser.read_type_a("bms_state", new unitree_go_msgs.msg.dds.BmsStatePubSubType(), data.getBmsState());

      ser.read_type_f("foot_force", data.getFootForce());
      ser.read_type_f("foot_force_est", data.getFootForceEst());
      data.setTick(ser.read_type_4("tick"));
      ser.read_type_f("wireless_remote", data.getWirelessRemote());
      data.setBitFlag(ser.read_type_9("bit_flag"));
      data.setAdcReel(ser.read_type_5("adc_reel"));
      data.setTemperatureNtc1(ser.read_type_9("temperature_ntc1"));
      data.setTemperatureNtc2(ser.read_type_9("temperature_ntc2"));
      data.setPowerV(ser.read_type_5("power_v"));
      data.setPowerA(ser.read_type_5("power_a"));
      data.setReserve(ser.read_type_4("reserve"));
      data.setCrc(ser.read_type_4("crc"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.LowState src, unitree_go_msgs.msg.dds.LowState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.LowState createData()
   {
      return new unitree_go_msgs.msg.dds.LowState();
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
   
   public void serialize(unitree_go_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.LowState src, unitree_go_msgs.msg.dds.LowState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LowStatePubSubType newInstance()
   {
      return new LowStatePubSubType();
   }
}

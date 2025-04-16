package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandState" defined in "HandState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandState_.idl instead.
*
*/
public class HandStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.HandState>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::HandState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "380fcd5b2fa73086217e178d8808485177194bd07828931fb3cbba5ef3f8f892";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.HandState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.HandState data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += unitree_h_one_msgs.msg.dds.MotorStatePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += unitree_h_one_msgs.msg.dds.IMUStatePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += unitree_h_one_msgs.msg.dds.PressSensorStatePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.HandState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.HandState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getMotorState().size(); ++i0)
      {
          current_alignment += unitree_h_one_msgs.msg.dds.MotorStatePubSubType.getCdrSerializedSize(data.getMotorState().get(i0), current_alignment);}

      current_alignment += unitree_h_one_msgs.msg.dds.IMUStatePubSubType.getCdrSerializedSize(data.getImuState(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPressSensorState().size(); ++i0)
      {
          current_alignment += unitree_h_one_msgs.msg.dds.PressSensorStatePubSubType.getCdrSerializedSize(data.getPressSensorState().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.HandState data, us.ihmc.idl.CDR cdr)
   {
      if(data.getMotorState().size() <= 100)
      cdr.write_type_e(data.getMotorState());else
          throw new RuntimeException("motor_state field exceeds the maximum length: %d > %d".formatted(data.getMotorState().size(), 100));

      unitree_h_one_msgs.msg.dds.IMUStatePubSubType.write(data.getImuState(), cdr);
      if(data.getPressSensorState().size() <= 100)
      cdr.write_type_e(data.getPressSensorState());else
          throw new RuntimeException("press_sensor_state field exceeds the maximum length: %d > %d".formatted(data.getPressSensorState().size(), 100));

      cdr.write_type_5(data.getPowerV());

      cdr.write_type_5(data.getPowerA());

   }

   public static void read(unitree_h_one_msgs.msg.dds.HandState data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getMotorState());	
      unitree_h_one_msgs.msg.dds.IMUStatePubSubType.read(data.getImuState(), cdr);	
      cdr.read_type_e(data.getPressSensorState());	
      data.setPowerV(cdr.read_type_5());
      	
      data.setPowerA(cdr.read_type_5());
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.HandState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("motor_state", data.getMotorState());
      ser.write_type_a("imu_state", new unitree_h_one_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      ser.write_type_e("press_sensor_state", data.getPressSensorState());
      ser.write_type_5("power_v", data.getPowerV());
      ser.write_type_5("power_a", data.getPowerA());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.HandState data)
   {
      ser.read_type_e("motor_state", data.getMotorState());
      ser.read_type_a("imu_state", new unitree_h_one_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      ser.read_type_e("press_sensor_state", data.getPressSensorState());
      data.setPowerV(ser.read_type_5("power_v"));
      data.setPowerA(ser.read_type_5("power_a"));
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.HandState src, unitree_h_one_msgs.msg.dds.HandState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.HandState createData()
   {
      return new unitree_h_one_msgs.msg.dds.HandState();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.HandState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.HandState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.HandState src, unitree_h_one_msgs.msg.dds.HandState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandStatePubSubType newInstance()
   {
      return new HandStatePubSubType();
   }
}

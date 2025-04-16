package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LowState" defined in "LowState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LowState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LowState_.idl instead.
*
*/
public class LowStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.LowState>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::LowState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "643ad4b441e0bddf6db6bb226ff46d11cfb33a87649421c2e158a2eefc6668cf";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.LowState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.LowState data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += unitree_h_one_msgs.msg.dds.IMUStatePubSubType.getMaxCdrSerializedSize(current_alignment);

      for(int i0 = 0; i0 < (35); ++i0)
      {
          current_alignment += unitree_h_one_msgs.msg.dds.MotorStatePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += ((40) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.LowState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.LowState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += unitree_h_one_msgs.msg.dds.IMUStatePubSubType.getCdrSerializedSize(data.getImuState(), current_alignment);

      for(int i0 = 0; i0 < data.getMotorState().length; ++i0)
      {
              current_alignment += unitree_h_one_msgs.msg.dds.MotorStatePubSubType.getCdrSerializedSize(data.getMotorState()[i0], current_alignment);
      }
      current_alignment += ((40) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getModePr());

      cdr.write_type_9(data.getModeMachine());

      cdr.write_type_4(data.getTick());

      unitree_h_one_msgs.msg.dds.IMUStatePubSubType.write(data.getImuState(), cdr);
      for(int i0 = 0; i0 < data.getMotorState().length; ++i0)
      {
        	unitree_h_one_msgs.msg.dds.MotorStatePubSubType.write(data.getMotorState()[i0], cdr);		
      }

      for(int i0 = 0; i0 < data.getWirelessRemote().length; ++i0)
      {
        	cdr.write_type_9(data.getWirelessRemote()[i0]);	
      }

      cdr.write_type_4(data.getCrc());

   }

   public static void read(unitree_h_one_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      data.setModePr(cdr.read_type_9());
      	
      data.setModeMachine(cdr.read_type_9());
      	
      data.setTick(cdr.read_type_4());
      	
      unitree_h_one_msgs.msg.dds.IMUStatePubSubType.read(data.getImuState(), cdr);	
      for(int i0 = 0; i0 < data.getMotorState().length; ++i0)
      {
        	unitree_h_one_msgs.msg.dds.MotorStatePubSubType.read(data.getMotorState()[i0], cdr);	
      }
      	
      for(int i0 = 0; i0 < data.getWirelessRemote().length; ++i0)
      {
        	data.getWirelessRemote()[i0] = cdr.read_type_9();
        	
      }
      	
      data.setCrc(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.LowState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("mode_pr", data.getModePr());
      ser.write_type_9("mode_machine", data.getModeMachine());
      ser.write_type_4("tick", data.getTick());
      ser.write_type_a("imu_state", new unitree_h_one_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      ser.write_type_f("motor_state", new unitree_h_one_msgs.msg.dds.MotorStatePubSubType(), data.getMotorState());
      ser.write_type_f("wireless_remote", data.getWirelessRemote());
      ser.write_type_4("crc", data.getCrc());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.LowState data)
   {
      data.setModePr(ser.read_type_9("mode_pr"));
      data.setModeMachine(ser.read_type_9("mode_machine"));
      data.setTick(ser.read_type_4("tick"));
      ser.read_type_a("imu_state", new unitree_h_one_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      ser.read_type_f("motor_state", new unitree_h_one_msgs.msg.dds.MotorStatePubSubType(), data.getMotorState());
      ser.read_type_f("wireless_remote", data.getWirelessRemote());
      data.setCrc(ser.read_type_4("crc"));
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.LowState src, unitree_h_one_msgs.msg.dds.LowState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.LowState createData()
   {
      return new unitree_h_one_msgs.msg.dds.LowState();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.LowState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.LowState src, unitree_h_one_msgs.msg.dds.LowState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LowStatePubSubType newInstance()
   {
      return new LowStatePubSubType();
   }
}

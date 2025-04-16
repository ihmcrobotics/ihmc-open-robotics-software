package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SportModeState" defined in "SportModeState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SportModeState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SportModeState_.idl instead.
*
*/
public class SportModeStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.SportModeState>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::SportModeState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ade2b0ae309470cad9e845599f48c61bc5c17aa37370cd79195c6245d7400b30";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.SportModeState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.SportModeState data) throws java.io.IOException
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

      current_alignment += unitree_go_msgs.msg.dds.TimeSpecPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += unitree_go_msgs.msg.dds.IMUStatePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((4) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((4) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.SportModeState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.SportModeState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += unitree_go_msgs.msg.dds.TimeSpecPubSubType.getCdrSerializedSize(data.getStamp(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += unitree_go_msgs.msg.dds.IMUStatePubSubType.getCdrSerializedSize(data.getImuState(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((4) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((4) * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);
      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((12) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.SportModeState data, us.ihmc.idl.CDR cdr)
   {
      unitree_go_msgs.msg.dds.TimeSpecPubSubType.write(data.getStamp(), cdr);
      cdr.write_type_4(data.getErrorCode());

      unitree_go_msgs.msg.dds.IMUStatePubSubType.write(data.getImuState(), cdr);
      cdr.write_type_9(data.getMode());

      cdr.write_type_5(data.getProgress());

      cdr.write_type_9(data.getGaitType());

      cdr.write_type_5(data.getFootRaiseHeight());

      for(int i0 = 0; i0 < data.getPosition().length; ++i0)
      {
        	cdr.write_type_5(data.getPosition()[i0]);	
      }

      cdr.write_type_5(data.getBodyHeight());

      for(int i0 = 0; i0 < data.getVelocity().length; ++i0)
      {
        	cdr.write_type_5(data.getVelocity()[i0]);	
      }

      cdr.write_type_5(data.getYawSpeed());

      for(int i0 = 0; i0 < data.getRangeObstacle().length; ++i0)
      {
        	cdr.write_type_5(data.getRangeObstacle()[i0]);	
      }

      for(int i0 = 0; i0 < data.getFootForce().length; ++i0)
      {
        	cdr.write_type_1(data.getFootForce()[i0]);	
      }

      for(int i0 = 0; i0 < data.getFootPositionBody().length; ++i0)
      {
        	cdr.write_type_5(data.getFootPositionBody()[i0]);	
      }

      for(int i0 = 0; i0 < data.getFootSpeedBody().length; ++i0)
      {
        	cdr.write_type_5(data.getFootSpeedBody()[i0]);	
      }

   }

   public static void read(unitree_go_msgs.msg.dds.SportModeState data, us.ihmc.idl.CDR cdr)
   {
      unitree_go_msgs.msg.dds.TimeSpecPubSubType.read(data.getStamp(), cdr);	
      data.setErrorCode(cdr.read_type_4());
      	
      unitree_go_msgs.msg.dds.IMUStatePubSubType.read(data.getImuState(), cdr);	
      data.setMode(cdr.read_type_9());
      	
      data.setProgress(cdr.read_type_5());
      	
      data.setGaitType(cdr.read_type_9());
      	
      data.setFootRaiseHeight(cdr.read_type_5());
      	
      for(int i0 = 0; i0 < data.getPosition().length; ++i0)
      {
        	data.getPosition()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setBodyHeight(cdr.read_type_5());
      	
      for(int i0 = 0; i0 < data.getVelocity().length; ++i0)
      {
        	data.getVelocity()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setYawSpeed(cdr.read_type_5());
      	
      for(int i0 = 0; i0 < data.getRangeObstacle().length; ++i0)
      {
        	data.getRangeObstacle()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getFootForce().length; ++i0)
      {
        	data.getFootForce()[i0] = cdr.read_type_1();
        	
      }
      	
      for(int i0 = 0; i0 < data.getFootPositionBody().length; ++i0)
      {
        	data.getFootPositionBody()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getFootSpeedBody().length; ++i0)
      {
        	data.getFootSpeedBody()[i0] = cdr.read_type_5();
        	
      }
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.SportModeState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("stamp", new unitree_go_msgs.msg.dds.TimeSpecPubSubType(), data.getStamp());

      ser.write_type_4("error_code", data.getErrorCode());
      ser.write_type_a("imu_state", new unitree_go_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      ser.write_type_9("mode", data.getMode());
      ser.write_type_5("progress", data.getProgress());
      ser.write_type_9("gait_type", data.getGaitType());
      ser.write_type_5("foot_raise_height", data.getFootRaiseHeight());
      ser.write_type_f("position", data.getPosition());
      ser.write_type_5("body_height", data.getBodyHeight());
      ser.write_type_f("velocity", data.getVelocity());
      ser.write_type_5("yaw_speed", data.getYawSpeed());
      ser.write_type_f("range_obstacle", data.getRangeObstacle());
      ser.write_type_f("foot_force", data.getFootForce());
      ser.write_type_f("foot_position_body", data.getFootPositionBody());
      ser.write_type_f("foot_speed_body", data.getFootSpeedBody());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.SportModeState data)
   {
      ser.read_type_a("stamp", new unitree_go_msgs.msg.dds.TimeSpecPubSubType(), data.getStamp());

      data.setErrorCode(ser.read_type_4("error_code"));
      ser.read_type_a("imu_state", new unitree_go_msgs.msg.dds.IMUStatePubSubType(), data.getImuState());

      data.setMode(ser.read_type_9("mode"));
      data.setProgress(ser.read_type_5("progress"));
      data.setGaitType(ser.read_type_9("gait_type"));
      data.setFootRaiseHeight(ser.read_type_5("foot_raise_height"));
      ser.read_type_f("position", data.getPosition());
      data.setBodyHeight(ser.read_type_5("body_height"));
      ser.read_type_f("velocity", data.getVelocity());
      data.setYawSpeed(ser.read_type_5("yaw_speed"));
      ser.read_type_f("range_obstacle", data.getRangeObstacle());
      ser.read_type_f("foot_force", data.getFootForce());
      ser.read_type_f("foot_position_body", data.getFootPositionBody());
      ser.read_type_f("foot_speed_body", data.getFootSpeedBody());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.SportModeState src, unitree_go_msgs.msg.dds.SportModeState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.SportModeState createData()
   {
      return new unitree_go_msgs.msg.dds.SportModeState();
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
   
   public void serialize(unitree_go_msgs.msg.dds.SportModeState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.SportModeState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.SportModeState src, unitree_go_msgs.msg.dds.SportModeState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SportModeStatePubSubType newInstance()
   {
      return new SportModeStatePubSubType();
   }
}

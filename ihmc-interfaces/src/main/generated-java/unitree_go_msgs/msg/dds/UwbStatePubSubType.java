package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "UwbState" defined in "UwbState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from UwbState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit UwbState_.idl instead.
*
*/
public class UwbStatePubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.UwbState>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::UwbState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "244041cfbbfe4c719a4aa77c8d37fcf7bb9038376322abf1008919825e2c781d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.UwbState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.UwbState data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.UwbState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.UwbState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.UwbState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getVersion().length; ++i0)
      {
        	cdr.write_type_9(data.getVersion()[i0]);	
      }

      cdr.write_type_9(data.getChannel());

      cdr.write_type_9(data.getJoyMode());

      cdr.write_type_5(data.getOrientationEst());

      cdr.write_type_5(data.getPitchEst());

      cdr.write_type_5(data.getDistanceEst());

      cdr.write_type_5(data.getYawEst());

      cdr.write_type_5(data.getTagRoll());

      cdr.write_type_5(data.getTagPitch());

      cdr.write_type_5(data.getTagYaw());

      cdr.write_type_5(data.getBaseRoll());

      cdr.write_type_5(data.getBasePitch());

      cdr.write_type_5(data.getBaseYaw());

      for(int i0 = 0; i0 < data.getJoystick().length; ++i0)
      {
        	cdr.write_type_5(data.getJoystick()[i0]);	
      }

      cdr.write_type_9(data.getErrorState());

      cdr.write_type_9(data.getButtons());

      cdr.write_type_9(data.getEnabledFromApp());

   }

   public static void read(unitree_go_msgs.msg.dds.UwbState data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getVersion().length; ++i0)
      {
        	data.getVersion()[i0] = cdr.read_type_9();
        	
      }
      	
      data.setChannel(cdr.read_type_9());
      	
      data.setJoyMode(cdr.read_type_9());
      	
      data.setOrientationEst(cdr.read_type_5());
      	
      data.setPitchEst(cdr.read_type_5());
      	
      data.setDistanceEst(cdr.read_type_5());
      	
      data.setYawEst(cdr.read_type_5());
      	
      data.setTagRoll(cdr.read_type_5());
      	
      data.setTagPitch(cdr.read_type_5());
      	
      data.setTagYaw(cdr.read_type_5());
      	
      data.setBaseRoll(cdr.read_type_5());
      	
      data.setBasePitch(cdr.read_type_5());
      	
      data.setBaseYaw(cdr.read_type_5());
      	
      for(int i0 = 0; i0 < data.getJoystick().length; ++i0)
      {
        	data.getJoystick()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setErrorState(cdr.read_type_9());
      	
      data.setButtons(cdr.read_type_9());
      	
      data.setEnabledFromApp(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.UwbState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("version", data.getVersion());
      ser.write_type_9("channel", data.getChannel());
      ser.write_type_9("joy_mode", data.getJoyMode());
      ser.write_type_5("orientation_est", data.getOrientationEst());
      ser.write_type_5("pitch_est", data.getPitchEst());
      ser.write_type_5("distance_est", data.getDistanceEst());
      ser.write_type_5("yaw_est", data.getYawEst());
      ser.write_type_5("tag_roll", data.getTagRoll());
      ser.write_type_5("tag_pitch", data.getTagPitch());
      ser.write_type_5("tag_yaw", data.getTagYaw());
      ser.write_type_5("base_roll", data.getBaseRoll());
      ser.write_type_5("base_pitch", data.getBasePitch());
      ser.write_type_5("base_yaw", data.getBaseYaw());
      ser.write_type_f("joystick", data.getJoystick());
      ser.write_type_9("error_state", data.getErrorState());
      ser.write_type_9("buttons", data.getButtons());
      ser.write_type_9("enabled_from_app", data.getEnabledFromApp());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.UwbState data)
   {
      ser.read_type_f("version", data.getVersion());
      data.setChannel(ser.read_type_9("channel"));
      data.setJoyMode(ser.read_type_9("joy_mode"));
      data.setOrientationEst(ser.read_type_5("orientation_est"));
      data.setPitchEst(ser.read_type_5("pitch_est"));
      data.setDistanceEst(ser.read_type_5("distance_est"));
      data.setYawEst(ser.read_type_5("yaw_est"));
      data.setTagRoll(ser.read_type_5("tag_roll"));
      data.setTagPitch(ser.read_type_5("tag_pitch"));
      data.setTagYaw(ser.read_type_5("tag_yaw"));
      data.setBaseRoll(ser.read_type_5("base_roll"));
      data.setBasePitch(ser.read_type_5("base_pitch"));
      data.setBaseYaw(ser.read_type_5("base_yaw"));
      ser.read_type_f("joystick", data.getJoystick());
      data.setErrorState(ser.read_type_9("error_state"));
      data.setButtons(ser.read_type_9("buttons"));
      data.setEnabledFromApp(ser.read_type_9("enabled_from_app"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.UwbState src, unitree_go_msgs.msg.dds.UwbState dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.UwbState createData()
   {
      return new unitree_go_msgs.msg.dds.UwbState();
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
   
   public void serialize(unitree_go_msgs.msg.dds.UwbState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.UwbState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.UwbState src, unitree_go_msgs.msg.dds.UwbState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public UwbStatePubSubType newInstance()
   {
      return new UwbStatePubSubType();
   }
}

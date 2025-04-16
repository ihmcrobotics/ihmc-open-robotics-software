package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SportModeCmd" defined in "SportModeCmd_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SportModeCmd_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SportModeCmd_.idl instead.
*
*/
public class SportModeCmdPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.SportModeCmd>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::SportModeCmd_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bfbd905004845e3188d3582c38bf0f83a6cc2f10ee91986a88f55024d804835d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.SportModeCmd data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.SportModeCmd data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += unitree_go_msgs.msg.dds.BmsCmdPubSubType.getMaxCdrSerializedSize(current_alignment);

      for(int i0 = 0; i0 < (30); ++i0)
      {
          current_alignment += unitree_go_msgs.msg.dds.PathPointPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.SportModeCmd data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.SportModeCmd data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((3) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((2) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += unitree_go_msgs.msg.dds.BmsCmdPubSubType.getCdrSerializedSize(data.getBmsCmd(), current_alignment);

      for(int i0 = 0; i0 < data.getPathPoint().length; ++i0)
      {
              current_alignment += unitree_go_msgs.msg.dds.PathPointPubSubType.getCdrSerializedSize(data.getPathPoint()[i0], current_alignment);
      }

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.SportModeCmd data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getMode());

      cdr.write_type_9(data.getGaitType());

      cdr.write_type_9(data.getSpeedLevel());

      cdr.write_type_5(data.getFootRaiseHeight());

      cdr.write_type_5(data.getBodyHeight());

      for(int i0 = 0; i0 < data.getPosition().length; ++i0)
      {
        	cdr.write_type_5(data.getPosition()[i0]);	
      }

      for(int i0 = 0; i0 < data.getEuler().length; ++i0)
      {
        	cdr.write_type_5(data.getEuler()[i0]);	
      }

      for(int i0 = 0; i0 < data.getVelocity().length; ++i0)
      {
        	cdr.write_type_5(data.getVelocity()[i0]);	
      }

      cdr.write_type_5(data.getYawSpeed());

      unitree_go_msgs.msg.dds.BmsCmdPubSubType.write(data.getBmsCmd(), cdr);
      for(int i0 = 0; i0 < data.getPathPoint().length; ++i0)
      {
        	unitree_go_msgs.msg.dds.PathPointPubSubType.write(data.getPathPoint()[i0], cdr);		
      }

   }

   public static void read(unitree_go_msgs.msg.dds.SportModeCmd data, us.ihmc.idl.CDR cdr)
   {
      data.setMode(cdr.read_type_9());
      	
      data.setGaitType(cdr.read_type_9());
      	
      data.setSpeedLevel(cdr.read_type_9());
      	
      data.setFootRaiseHeight(cdr.read_type_5());
      	
      data.setBodyHeight(cdr.read_type_5());
      	
      for(int i0 = 0; i0 < data.getPosition().length; ++i0)
      {
        	data.getPosition()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getEuler().length; ++i0)
      {
        	data.getEuler()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getVelocity().length; ++i0)
      {
        	data.getVelocity()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setYawSpeed(cdr.read_type_5());
      	
      unitree_go_msgs.msg.dds.BmsCmdPubSubType.read(data.getBmsCmd(), cdr);	
      for(int i0 = 0; i0 < data.getPathPoint().length; ++i0)
      {
        	unitree_go_msgs.msg.dds.PathPointPubSubType.read(data.getPathPoint()[i0], cdr);	
      }
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.SportModeCmd data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("mode", data.getMode());
      ser.write_type_9("gait_type", data.getGaitType());
      ser.write_type_9("speed_level", data.getSpeedLevel());
      ser.write_type_5("foot_raise_height", data.getFootRaiseHeight());
      ser.write_type_5("body_height", data.getBodyHeight());
      ser.write_type_f("position", data.getPosition());
      ser.write_type_f("euler", data.getEuler());
      ser.write_type_f("velocity", data.getVelocity());
      ser.write_type_5("yaw_speed", data.getYawSpeed());
      ser.write_type_a("bms_cmd", new unitree_go_msgs.msg.dds.BmsCmdPubSubType(), data.getBmsCmd());

      ser.write_type_f("path_point", new unitree_go_msgs.msg.dds.PathPointPubSubType(), data.getPathPoint());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.SportModeCmd data)
   {
      data.setMode(ser.read_type_9("mode"));
      data.setGaitType(ser.read_type_9("gait_type"));
      data.setSpeedLevel(ser.read_type_9("speed_level"));
      data.setFootRaiseHeight(ser.read_type_5("foot_raise_height"));
      data.setBodyHeight(ser.read_type_5("body_height"));
      ser.read_type_f("position", data.getPosition());
      ser.read_type_f("euler", data.getEuler());
      ser.read_type_f("velocity", data.getVelocity());
      data.setYawSpeed(ser.read_type_5("yaw_speed"));
      ser.read_type_a("bms_cmd", new unitree_go_msgs.msg.dds.BmsCmdPubSubType(), data.getBmsCmd());

      ser.read_type_f("path_point", new unitree_go_msgs.msg.dds.PathPointPubSubType(), data.getPathPoint());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.SportModeCmd src, unitree_go_msgs.msg.dds.SportModeCmd dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.SportModeCmd createData()
   {
      return new unitree_go_msgs.msg.dds.SportModeCmd();
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
   
   public void serialize(unitree_go_msgs.msg.dds.SportModeCmd data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.SportModeCmd data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.SportModeCmd src, unitree_go_msgs.msg.dds.SportModeCmd dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SportModeCmdPubSubType newInstance()
   {
      return new SportModeCmdPubSubType();
   }
}

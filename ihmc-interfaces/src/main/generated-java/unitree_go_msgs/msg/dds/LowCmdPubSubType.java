package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LowCmd" defined in "LowCmd_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LowCmd_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LowCmd_.idl instead.
*
*/
public class LowCmdPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.LowCmd>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::LowCmd_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "650be524ccd9dd54ee5848a72ec48ac2a8e8d30ee579d3b2acb45527dec24efb";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.LowCmd data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.LowCmd data) throws java.io.IOException
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

      for(int i0 = 0; i0 < (20); ++i0)
      {
          current_alignment += unitree_go_msgs.msg.dds.MotorCmdPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += unitree_go_msgs.msg.dds.BmsCmdPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ((40) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((12) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.LowCmd data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.LowCmd data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      for(int i0 = 0; i0 < data.getMotorCmd().length; ++i0)
      {
              current_alignment += unitree_go_msgs.msg.dds.MotorCmdPubSubType.getCdrSerializedSize(data.getMotorCmd()[i0], current_alignment);
      }
      current_alignment += unitree_go_msgs.msg.dds.BmsCmdPubSubType.getCdrSerializedSize(data.getBmsCmd(), current_alignment);

      current_alignment += ((40) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += ((12) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += ((2) * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getHead().length; ++i0)
      {
        	cdr.write_type_9(data.getHead()[i0]);	
      }

      cdr.write_type_9(data.getLevelFlag());

      cdr.write_type_9(data.getFrameReserve());

      cdr.write_type_3(data.getBandwidth());

      for(int i0 = 0; i0 < data.getMotorCmd().length; ++i0)
      {
        	unitree_go_msgs.msg.dds.MotorCmdPubSubType.write(data.getMotorCmd()[i0], cdr);		
      }

      unitree_go_msgs.msg.dds.BmsCmdPubSubType.write(data.getBmsCmd(), cdr);
      for(int i0 = 0; i0 < data.getWirelessRemote().length; ++i0)
      {
        	cdr.write_type_9(data.getWirelessRemote()[i0]);	
      }

      for(int i0 = 0; i0 < data.getLed().length; ++i0)
      {
        	cdr.write_type_9(data.getLed()[i0]);	
      }

      for(int i0 = 0; i0 < data.getFan().length; ++i0)
      {
        	cdr.write_type_9(data.getFan()[i0]);	
      }

      cdr.write_type_9(data.getGpio());

      cdr.write_type_4(data.getReserve());

      cdr.write_type_4(data.getCrc());

   }

   public static void read(unitree_go_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      for(int i0 = 0; i0 < data.getHead().length; ++i0)
      {
        	data.getHead()[i0] = cdr.read_type_9();
        	
      }
      	
      data.setLevelFlag(cdr.read_type_9());
      	
      data.setFrameReserve(cdr.read_type_9());
      	
      data.setBandwidth(cdr.read_type_3());
      	
      for(int i0 = 0; i0 < data.getMotorCmd().length; ++i0)
      {
        	unitree_go_msgs.msg.dds.MotorCmdPubSubType.read(data.getMotorCmd()[i0], cdr);	
      }
      	
      unitree_go_msgs.msg.dds.BmsCmdPubSubType.read(data.getBmsCmd(), cdr);	
      for(int i0 = 0; i0 < data.getWirelessRemote().length; ++i0)
      {
        	data.getWirelessRemote()[i0] = cdr.read_type_9();
        	
      }
      	
      for(int i0 = 0; i0 < data.getLed().length; ++i0)
      {
        	data.getLed()[i0] = cdr.read_type_9();
        	
      }
      	
      for(int i0 = 0; i0 < data.getFan().length; ++i0)
      {
        	data.getFan()[i0] = cdr.read_type_9();
        	
      }
      	
      data.setGpio(cdr.read_type_9());
      	
      data.setReserve(cdr.read_type_4());
      	
      data.setCrc(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.LowCmd data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_f("head", data.getHead());
      ser.write_type_9("level_flag", data.getLevelFlag());
      ser.write_type_9("frame_reserve", data.getFrameReserve());
      ser.write_type_3("bandwidth", data.getBandwidth());
      ser.write_type_f("motor_cmd", new unitree_go_msgs.msg.dds.MotorCmdPubSubType(), data.getMotorCmd());
      ser.write_type_a("bms_cmd", new unitree_go_msgs.msg.dds.BmsCmdPubSubType(), data.getBmsCmd());

      ser.write_type_f("wireless_remote", data.getWirelessRemote());
      ser.write_type_f("led", data.getLed());
      ser.write_type_f("fan", data.getFan());
      ser.write_type_9("gpio", data.getGpio());
      ser.write_type_4("reserve", data.getReserve());
      ser.write_type_4("crc", data.getCrc());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.LowCmd data)
   {
      ser.read_type_f("head", data.getHead());
      data.setLevelFlag(ser.read_type_9("level_flag"));
      data.setFrameReserve(ser.read_type_9("frame_reserve"));
      data.setBandwidth(ser.read_type_3("bandwidth"));
      ser.read_type_f("motor_cmd", new unitree_go_msgs.msg.dds.MotorCmdPubSubType(), data.getMotorCmd());
      ser.read_type_a("bms_cmd", new unitree_go_msgs.msg.dds.BmsCmdPubSubType(), data.getBmsCmd());

      ser.read_type_f("wireless_remote", data.getWirelessRemote());
      ser.read_type_f("led", data.getLed());
      ser.read_type_f("fan", data.getFan());
      data.setGpio(ser.read_type_9("gpio"));
      data.setReserve(ser.read_type_4("reserve"));
      data.setCrc(ser.read_type_4("crc"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.LowCmd src, unitree_go_msgs.msg.dds.LowCmd dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.LowCmd createData()
   {
      return new unitree_go_msgs.msg.dds.LowCmd();
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
   
   public void serialize(unitree_go_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.LowCmd src, unitree_go_msgs.msg.dds.LowCmd dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LowCmdPubSubType newInstance()
   {
      return new LowCmdPubSubType();
   }
}

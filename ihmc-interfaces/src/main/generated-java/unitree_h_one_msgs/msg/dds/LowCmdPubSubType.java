package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LowCmd" defined in "LowCmd_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LowCmd_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LowCmd_.idl instead.
*
*/
public class LowCmdPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.LowCmd>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::LowCmd_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ac190ef8df8491986cb316265a28210a6ac84b5f487c472453c90dd719665f82";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.LowCmd data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.LowCmd data) throws java.io.IOException
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

      for(int i0 = 0; i0 < (35); ++i0)
      {
          current_alignment += unitree_h_one_msgs.msg.dds.MotorCmdPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.LowCmd data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.LowCmd data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      for(int i0 = 0; i0 < data.getMotorCmd().length; ++i0)
      {
              current_alignment += unitree_h_one_msgs.msg.dds.MotorCmdPubSubType.getCdrSerializedSize(data.getMotorCmd()[i0], current_alignment);
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getModePr());

      cdr.write_type_9(data.getModeMachine());

      for(int i0 = 0; i0 < data.getMotorCmd().length; ++i0)
      {
        	unitree_h_one_msgs.msg.dds.MotorCmdPubSubType.write(data.getMotorCmd()[i0], cdr);		
      }

      cdr.write_type_4(data.getCrc());

   }

   public static void read(unitree_h_one_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      data.setModePr(cdr.read_type_9());
      	
      data.setModeMachine(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getMotorCmd().length; ++i0)
      {
        	unitree_h_one_msgs.msg.dds.MotorCmdPubSubType.read(data.getMotorCmd()[i0], cdr);	
      }
      	
      data.setCrc(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.LowCmd data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("mode_pr", data.getModePr());
      ser.write_type_9("mode_machine", data.getModeMachine());
      ser.write_type_f("motor_cmd", new unitree_h_one_msgs.msg.dds.MotorCmdPubSubType(), data.getMotorCmd());
      ser.write_type_4("crc", data.getCrc());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.LowCmd data)
   {
      data.setModePr(ser.read_type_9("mode_pr"));
      data.setModeMachine(ser.read_type_9("mode_machine"));
      ser.read_type_f("motor_cmd", new unitree_h_one_msgs.msg.dds.MotorCmdPubSubType(), data.getMotorCmd());
      data.setCrc(ser.read_type_4("crc"));
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.LowCmd src, unitree_h_one_msgs.msg.dds.LowCmd dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.LowCmd createData()
   {
      return new unitree_h_one_msgs.msg.dds.LowCmd();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.LowCmd data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.LowCmd src, unitree_h_one_msgs.msg.dds.LowCmd dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LowCmdPubSubType newInstance()
   {
      return new LowCmdPubSubType();
   }
}

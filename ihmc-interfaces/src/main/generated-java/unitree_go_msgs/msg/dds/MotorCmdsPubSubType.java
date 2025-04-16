package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MotorCmds" defined in "MotorCmds_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MotorCmds_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MotorCmds_.idl instead.
*
*/
public class MotorCmdsPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.MotorCmds>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::MotorCmds_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e5ae45e841905f666df02c5c871b163695af904749f6d9cf20562655a83b3199";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.MotorCmds data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.MotorCmds data) throws java.io.IOException
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
          current_alignment += unitree_go_msgs.msg.dds.MotorCmdPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorCmds data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorCmds data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getCmds().size(); ++i0)
      {
          current_alignment += unitree_go_msgs.msg.dds.MotorCmdPubSubType.getCdrSerializedSize(data.getCmds().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.MotorCmds data, us.ihmc.idl.CDR cdr)
   {
      if(data.getCmds().size() <= 100)
      cdr.write_type_e(data.getCmds());else
          throw new RuntimeException("cmds field exceeds the maximum length: %d > %d".formatted(data.getCmds().size(), 100));

   }

   public static void read(unitree_go_msgs.msg.dds.MotorCmds data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getCmds());	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.MotorCmds data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("cmds", data.getCmds());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.MotorCmds data)
   {
      ser.read_type_e("cmds", data.getCmds());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.MotorCmds src, unitree_go_msgs.msg.dds.MotorCmds dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.MotorCmds createData()
   {
      return new unitree_go_msgs.msg.dds.MotorCmds();
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
   
   public void serialize(unitree_go_msgs.msg.dds.MotorCmds data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.MotorCmds data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.MotorCmds src, unitree_go_msgs.msg.dds.MotorCmds dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MotorCmdsPubSubType newInstance()
   {
      return new MotorCmdsPubSubType();
   }
}

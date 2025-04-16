package unitree_h_one_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandCmd" defined in "HandCmd_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandCmd_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandCmd_.idl instead.
*
*/
public class HandCmdPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_h_one_msgs.msg.dds.HandCmd>
{
   public static final java.lang.String name = "unitree_h_one_msgs::msg::dds_::HandCmd_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "50c51535713462cbf191eb127de52a55733098af1ed9e7a8622d151169f76823";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_h_one_msgs.msg.dds.HandCmd data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_h_one_msgs.msg.dds.HandCmd data) throws java.io.IOException
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
          current_alignment += unitree_h_one_msgs.msg.dds.MotorCmdPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.HandCmd data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_h_one_msgs.msg.dds.HandCmd data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getMotorCmd().size(); ++i0)
      {
          current_alignment += unitree_h_one_msgs.msg.dds.MotorCmdPubSubType.getCdrSerializedSize(data.getMotorCmd().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_h_one_msgs.msg.dds.HandCmd data, us.ihmc.idl.CDR cdr)
   {
      if(data.getMotorCmd().size() <= 100)
      cdr.write_type_e(data.getMotorCmd());else
          throw new RuntimeException("motor_cmd field exceeds the maximum length: %d > %d".formatted(data.getMotorCmd().size(), 100));

   }

   public static void read(unitree_h_one_msgs.msg.dds.HandCmd data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getMotorCmd());	

   }

   @Override
   public final void serialize(unitree_h_one_msgs.msg.dds.HandCmd data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("motor_cmd", data.getMotorCmd());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_h_one_msgs.msg.dds.HandCmd data)
   {
      ser.read_type_e("motor_cmd", data.getMotorCmd());
   }

   public static void staticCopy(unitree_h_one_msgs.msg.dds.HandCmd src, unitree_h_one_msgs.msg.dds.HandCmd dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_h_one_msgs.msg.dds.HandCmd createData()
   {
      return new unitree_h_one_msgs.msg.dds.HandCmd();
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
   
   public void serialize(unitree_h_one_msgs.msg.dds.HandCmd data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_h_one_msgs.msg.dds.HandCmd data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_h_one_msgs.msg.dds.HandCmd src, unitree_h_one_msgs.msg.dds.HandCmd dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandCmdPubSubType newInstance()
   {
      return new HandCmdPubSubType();
   }
}

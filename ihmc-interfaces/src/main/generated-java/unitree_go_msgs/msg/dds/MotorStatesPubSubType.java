package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MotorStates" defined in "MotorStates_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MotorStates_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MotorStates_.idl instead.
*
*/
public class MotorStatesPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.MotorStates>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::MotorStates_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "68605829b79ccf7f08d7b4aed55786b47029a02c23ced70dc22d0b273b46263a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.MotorStates data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.MotorStates data) throws java.io.IOException
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
          current_alignment += unitree_go_msgs.msg.dds.MotorStatePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorStates data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.MotorStates data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getStates().size(); ++i0)
      {
          current_alignment += unitree_go_msgs.msg.dds.MotorStatePubSubType.getCdrSerializedSize(data.getStates().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.MotorStates data, us.ihmc.idl.CDR cdr)
   {
      if(data.getStates().size() <= 100)
      cdr.write_type_e(data.getStates());else
          throw new RuntimeException("states field exceeds the maximum length: %d > %d".formatted(data.getStates().size(), 100));

   }

   public static void read(unitree_go_msgs.msg.dds.MotorStates data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getStates());	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.MotorStates data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("states", data.getStates());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.MotorStates data)
   {
      ser.read_type_e("states", data.getStates());
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.MotorStates src, unitree_go_msgs.msg.dds.MotorStates dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.MotorStates createData()
   {
      return new unitree_go_msgs.msg.dds.MotorStates();
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
   
   public void serialize(unitree_go_msgs.msg.dds.MotorStates data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.MotorStates data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.MotorStates src, unitree_go_msgs.msg.dds.MotorStates dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MotorStatesPubSubType newInstance()
   {
      return new MotorStatesPubSubType();
   }
}

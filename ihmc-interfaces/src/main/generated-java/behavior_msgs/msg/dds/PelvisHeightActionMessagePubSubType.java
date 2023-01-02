package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PelvisHeightActionMessage" defined in "PelvisHeightActionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PelvisHeightActionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PelvisHeightActionMessage_.idl instead.
*
*/
public class PelvisHeightActionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.PelvisHeightActionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::PelvisHeightActionMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.PelvisHeightActionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.PelvisHeightActionMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PelvisHeightActionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.PelvisHeightActionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.PelvisHeightActionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getTrajectoryDuration());

      cdr.write_type_6(data.getHeightInWorld());

   }

   public static void read(behavior_msgs.msg.dds.PelvisHeightActionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setTrajectoryDuration(cdr.read_type_6());
      	
      data.setHeightInWorld(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.PelvisHeightActionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("trajectory_duration", data.getTrajectoryDuration());
      ser.write_type_6("height_in_world", data.getHeightInWorld());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.PelvisHeightActionMessage data)
   {
      data.setTrajectoryDuration(ser.read_type_6("trajectory_duration"));
      data.setHeightInWorld(ser.read_type_6("height_in_world"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.PelvisHeightActionMessage src, behavior_msgs.msg.dds.PelvisHeightActionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.PelvisHeightActionMessage createData()
   {
      return new behavior_msgs.msg.dds.PelvisHeightActionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.PelvisHeightActionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.PelvisHeightActionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.PelvisHeightActionMessage src, behavior_msgs.msg.dds.PelvisHeightActionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PelvisHeightActionMessagePubSubType newInstance()
   {
      return new PelvisHeightActionMessagePubSubType();
   }
}

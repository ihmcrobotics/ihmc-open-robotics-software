package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyBimanipulationActionStateMessage" defined in "WholeBodyBimanipulationActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyBimanipulationActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyBimanipulationActionStateMessage_.idl instead.
*
*/
public class WholeBodyBimanipulationActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::WholeBodyBimanipulationActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "754b3909120bb4c1bd613845eee108bfe01792910e43f404d564d2b0c51e7951";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getSolutionQuality());

   }

   public static void read(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	data.getJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setSolutionQuality(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_f("joint_angles", data.getJointAngles());
      ser.write_type_6("solution_quality", data.getSolutionQuality());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.WholeBodyBimanipulationActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_f("joint_angles", data.getJointAngles());
      data.setSolutionQuality(ser.read_type_6("solution_quality"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage src, behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage src, behavior_msgs.msg.dds.WholeBodyBimanipulationActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyBimanipulationActionStateMessagePubSubType newInstance()
   {
      return new WholeBodyBimanipulationActionStateMessagePubSubType();
   }
}

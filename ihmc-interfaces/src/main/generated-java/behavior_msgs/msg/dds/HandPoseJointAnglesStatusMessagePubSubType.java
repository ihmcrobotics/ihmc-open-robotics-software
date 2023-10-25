package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandPoseJointAnglesStatusMessage" defined in "HandPoseJointAnglesStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandPoseJointAnglesStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandPoseJointAnglesStatusMessage_.idl instead.
*
*/
public class HandPoseJointAnglesStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::HandPoseJointAnglesStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b092ea7ec784d497a76110f0b2699094ef7e8c6f05b514ee064c757abfd0876f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data) throws java.io.IOException
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

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getSolutionQuality());

   }

   public static void read(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	data.getJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setSolutionQuality(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_f("joint_angles", data.getJointAngles());
      ser.write_type_6("solution_quality", data.getSolutionQuality());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_f("joint_angles", data.getJointAngles());
      data.setSolutionQuality(ser.read_type_6("solution_quality"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage src, behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage createData()
   {
      return new behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage src, behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandPoseJointAnglesStatusMessagePubSubType newInstance()
   {
      return new HandPoseJointAnglesStatusMessagePubSubType();
   }
}

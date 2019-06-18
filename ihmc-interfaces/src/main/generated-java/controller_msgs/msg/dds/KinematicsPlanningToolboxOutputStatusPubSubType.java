package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsPlanningToolboxOutputStatus" defined in "KinematicsPlanningToolboxOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsPlanningToolboxOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsPlanningToolboxOutputStatus_.idl instead.
*
*/
public class KinematicsPlanningToolboxOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsPlanningToolboxOutputStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getKeyFrameTimes().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRobotConfigurations().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getCdrSerializedSize(data.getRobotConfigurations().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType.getCdrSerializedSize(data.getSuggestedControllerMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getPlanId());

      if(data.getKeyFrameTimes().size() <= 100)
      cdr.write_type_e(data.getKeyFrameTimes());else
          throw new RuntimeException("key_frame_times field exceeds the maximum length");

      if(data.getRobotConfigurations().size() <= 100)
      cdr.write_type_e(data.getRobotConfigurations());else
          throw new RuntimeException("robot_configurations field exceeds the maximum length");

      cdr.write_type_6(data.getSolutionQuality());

      controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType.write(data.getSuggestedControllerMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setPlanId(cdr.read_type_2());
      	
      cdr.read_type_e(data.getKeyFrameTimes());	
      cdr.read_type_e(data.getRobotConfigurations());	
      data.setSolutionQuality(cdr.read_type_6());
      	
      controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType.read(data.getSuggestedControllerMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("plan_id", data.getPlanId());
      ser.write_type_e("key_frame_times", data.getKeyFrameTimes());
      ser.write_type_e("robot_configurations", data.getRobotConfigurations());
      ser.write_type_6("solution_quality", data.getSolutionQuality());
      ser.write_type_a("suggested_controller_message", new controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType(), data.getSuggestedControllerMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setPlanId(ser.read_type_2("plan_id"));
      ser.read_type_e("key_frame_times", data.getKeyFrameTimes());
      ser.read_type_e("robot_configurations", data.getRobotConfigurations());
      data.setSolutionQuality(ser.read_type_6("solution_quality"));
      ser.read_type_a("suggested_controller_message", new controller_msgs.msg.dds.WholeBodyTrajectoryMessagePubSubType(), data.getSuggestedControllerMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus src, controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus createData()
   {
      return new controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus src, controller_msgs.msg.dds.KinematicsPlanningToolboxOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsPlanningToolboxOutputStatusPubSubType newInstance()
   {
      return new KinematicsPlanningToolboxOutputStatusPubSubType();
   }
}

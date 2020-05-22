package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyTrajectoryToolboxOutputStatus" defined in "WholeBodyTrajectoryToolboxOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyTrajectoryToolboxOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyTrajectoryToolboxOutputStatus_.idl instead.
*
*/
public class WholeBodyTrajectoryToolboxOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WholeBodyTrajectoryToolboxOutputStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (50 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getTrajectoryTimes().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRobotConfigurations().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getCdrSerializedSize(data.getRobotConfigurations().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_2(data.getPlanningResult());


      if(data.getTrajectoryTimes().size() <= 50)
      cdr.write_type_e(data.getTrajectoryTimes());else
          throw new RuntimeException("trajectory_times field exceeds the maximum length");


      if(data.getRobotConfigurations().size() <= 50)
      cdr.write_type_e(data.getRobotConfigurations());else
          throw new RuntimeException("robot_configurations field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setPlanningResult(cdr.read_type_2());
      	

      cdr.read_type_e(data.getTrajectoryTimes());	

      cdr.read_type_e(data.getRobotConfigurations());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_2("planning_result", data.getPlanningResult());

      ser.write_type_e("trajectory_times", data.getTrajectoryTimes());

      ser.write_type_e("robot_configurations", data.getRobotConfigurations());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setPlanningResult(ser.read_type_2("planning_result"));

      ser.read_type_e("trajectory_times", data.getTrajectoryTimes());

      ser.read_type_e("robot_configurations", data.getRobotConfigurations());
   }

   public static void staticCopy(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus src, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus createData()
   {
      return new controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus();
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
   
   public void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus src, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyTrajectoryToolboxOutputStatusPubSubType newInstance()
   {
      return new WholeBodyTrajectoryToolboxOutputStatusPubSubType();
   }
}

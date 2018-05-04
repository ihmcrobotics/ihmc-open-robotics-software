package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyTrajectoryToolboxConfigurationMessage" defined in "WholeBodyTrajectoryToolboxConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyTrajectoryToolboxConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyTrajectoryToolboxConfigurationMessage_.idl instead.
*
*/
public class WholeBodyTrajectoryToolboxConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WholeBodyTrajectoryToolboxConfigurationMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getCdrSerializedSize(data.getInitialConfiguration(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getNumberOfInitialGuesses());

      cdr.write_type_2(data.getMaximumExpansionSize());

      controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.write(data.getInitialConfiguration(), cdr);
   }

   public static void read(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setNumberOfInitialGuesses(cdr.read_type_2());
      	
      data.setMaximumExpansionSize(cdr.read_type_2());
      	
      controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.read(data.getInitialConfiguration(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("number_of_initial_guesses", data.getNumberOfInitialGuesses());
      ser.write_type_2("maximum_expansion_size", data.getMaximumExpansionSize());
      ser.write_type_a("initial_configuration", new controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType(), data.getInitialConfiguration());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setNumberOfInitialGuesses(ser.read_type_2("number_of_initial_guesses"));
      data.setMaximumExpansionSize(ser.read_type_2("maximum_expansion_size"));
      ser.read_type_a("initial_configuration", new controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType(), data.getInitialConfiguration());

   }

   public static void staticCopy(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage src, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage createData()
   {
      return new controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage src, controller_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyTrajectoryToolboxConfigurationMessagePubSubType newInstance()
   {
      return new WholeBodyTrajectoryToolboxConfigurationMessagePubSubType();
   }
}

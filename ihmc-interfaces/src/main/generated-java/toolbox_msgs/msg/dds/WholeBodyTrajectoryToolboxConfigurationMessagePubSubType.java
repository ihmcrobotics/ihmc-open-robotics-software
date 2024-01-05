package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyTrajectoryToolboxConfigurationMessage" defined in "WholeBodyTrajectoryToolboxConfigurationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyTrajectoryToolboxConfigurationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyTrajectoryToolboxConfigurationMessage_.idl instead.
*
*/
public class WholeBodyTrajectoryToolboxConfigurationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::WholeBodyTrajectoryToolboxConfigurationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3c9acd47fcc8735572ecf9b7b38062a10ba50f0c51d738a9c46359aa8b1117a2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data) throws java.io.IOException
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

      current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.getCdrSerializedSize(data.getInitialConfiguration(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getNumberOfInitialGuesses());

      cdr.write_type_2(data.getMaximumExpansionSize());

      toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.write(data.getInitialConfiguration(), cdr);
   }

   public static void read(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setNumberOfInitialGuesses(cdr.read_type_2());
      	
      data.setMaximumExpansionSize(cdr.read_type_2());
      	
      toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.read(data.getInitialConfiguration(), cdr);	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("number_of_initial_guesses", data.getNumberOfInitialGuesses());
      ser.write_type_2("maximum_expansion_size", data.getMaximumExpansionSize());
      ser.write_type_a("initial_configuration", new toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType(), data.getInitialConfiguration());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setNumberOfInitialGuesses(ser.read_type_2("number_of_initial_guesses"));
      data.setMaximumExpansionSize(ser.read_type_2("maximum_expansion_size"));
      ser.read_type_a("initial_configuration", new toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType(), data.getInitialConfiguration());

   }

   public static void staticCopy(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage src, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage createData()
   {
      return new toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage src, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyTrajectoryToolboxConfigurationMessagePubSubType newInstance()
   {
      return new WholeBodyTrajectoryToolboxConfigurationMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlanningStatistics" defined in "FootstepPlanningStatistics_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlanningStatistics_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlanningStatistics_.idl instead.
*
*/
public class FootstepPlanningStatisticsPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlanningStatistics>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlanningStatistics_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlanningStatistics data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlanningStatistics data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (16 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanningStatistics data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlanningStatistics data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getRejectionFractions().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlanningStatistics data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getTimeTaken());

      cdr.write_type_2(data.getNumberOfStepsConsidered());

      if(data.getRejectionFractions().size() <= 16)
      cdr.write_type_e(data.getRejectionFractions());else
          throw new RuntimeException("rejection_fractions field exceeds the maximum length");

      cdr.write_type_6(data.getFractionOfRejectedSteps());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlanningStatistics data, us.ihmc.idl.CDR cdr)
   {
      data.setTimeTaken(cdr.read_type_6());
      	
      data.setNumberOfStepsConsidered(cdr.read_type_2());
      	
      cdr.read_type_e(data.getRejectionFractions());	
      data.setFractionOfRejectedSteps(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlanningStatistics data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("time_taken", data.getTimeTaken());
      ser.write_type_2("number_of_steps_considered", data.getNumberOfStepsConsidered());
      ser.write_type_e("rejection_fractions", data.getRejectionFractions());
      ser.write_type_6("fraction_of_rejected_steps", data.getFractionOfRejectedSteps());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlanningStatistics data)
   {
      data.setTimeTaken(ser.read_type_6("time_taken"));
      data.setNumberOfStepsConsidered(ser.read_type_2("number_of_steps_considered"));
      ser.read_type_e("rejection_fractions", data.getRejectionFractions());
      data.setFractionOfRejectedSteps(ser.read_type_6("fraction_of_rejected_steps"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlanningStatistics src, controller_msgs.msg.dds.FootstepPlanningStatistics dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlanningStatistics createData()
   {
      return new controller_msgs.msg.dds.FootstepPlanningStatistics();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlanningStatistics data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlanningStatistics data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlanningStatistics src, controller_msgs.msg.dds.FootstepPlanningStatistics dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlanningStatisticsPubSubType newInstance()
   {
      return new FootstepPlanningStatisticsPubSubType();
   }
}

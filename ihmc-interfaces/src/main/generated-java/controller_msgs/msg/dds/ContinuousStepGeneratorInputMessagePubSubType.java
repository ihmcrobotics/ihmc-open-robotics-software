package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ContinuousStepGeneratorInputMessage" defined in "ContinuousStepGeneratorInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ContinuousStepGeneratorInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ContinuousStepGeneratorInputMessage_.idl instead.
*
*/
public class ContinuousStepGeneratorInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ContinuousStepGeneratorInputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9b769610caee25d78da0240df2032f7cc0ebe07f190f740564d1897cf1f1ead1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getWalk());

      cdr.write_type_6(data.getForwardVelocity());

      cdr.write_type_6(data.getLateralVelocity());

      cdr.write_type_6(data.getTurnVelocity());

      cdr.write_type_7(data.getUnitVelocities());

   }

   public static void read(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setWalk(cdr.read_type_7());
      	
      data.setForwardVelocity(cdr.read_type_6());
      	
      data.setLateralVelocity(cdr.read_type_6());
      	
      data.setTurnVelocity(cdr.read_type_6());
      	
      data.setUnitVelocities(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("walk", data.getWalk());
      ser.write_type_6("forward_velocity", data.getForwardVelocity());
      ser.write_type_6("lateral_velocity", data.getLateralVelocity());
      ser.write_type_6("turn_velocity", data.getTurnVelocity());
      ser.write_type_7("unit_velocities", data.getUnitVelocities());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setWalk(ser.read_type_7("walk"));
      data.setForwardVelocity(ser.read_type_6("forward_velocity"));
      data.setLateralVelocity(ser.read_type_6("lateral_velocity"));
      data.setTurnVelocity(ser.read_type_6("turn_velocity"));
      data.setUnitVelocities(ser.read_type_7("unit_velocities"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage src, controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage createData()
   {
      return new controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage src, controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ContinuousStepGeneratorInputMessagePubSubType newInstance()
   {
      return new ContinuousStepGeneratorInputMessagePubSubType();
   }
}

package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CrocoddylSolverTrajectoryMessage" defined in "CrocoddylSolverTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CrocoddylSolverTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CrocoddylSolverTrajectoryMessage_.idl instead.
*
*/
public class CrocoddylSolverTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CrocoddylSolverTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "107b3ddabe8f9260b133c69c6c097d80f80af085d5300b1251c0a9c7b98cb6e5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CrocoddylTimeIntervalMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CrocoddylStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CrocoddylControlMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getIntervals().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CrocoddylTimeIntervalMessagePubSubType.getCdrSerializedSize(data.getIntervals().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getStateTrajectory().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CrocoddylStateMessagePubSubType.getCdrSerializedSize(data.getStateTrajectory().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getControlTrajectory().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.CrocoddylControlMessagePubSubType.getCdrSerializedSize(data.getControlTrajectory().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);
      if(data.getIntervals().size() <= 100)
      cdr.write_type_e(data.getIntervals());else
          throw new RuntimeException("intervals field exceeds the maximum length");

      if(data.getStateTrajectory().size() <= 100)
      cdr.write_type_e(data.getStateTrajectory());else
          throw new RuntimeException("state_trajectory field exceeds the maximum length");

      if(data.getControlTrajectory().size() <= 100)
      cdr.write_type_e(data.getControlTrajectory());else
          throw new RuntimeException("control_trajectory field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);	
      cdr.read_type_e(data.getIntervals());	
      cdr.read_type_e(data.getStateTrajectory());	
      cdr.read_type_e(data.getControlTrajectory());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_e("intervals", data.getIntervals());
      ser.write_type_e("state_trajectory", data.getStateTrajectory());
      ser.write_type_e("control_trajectory", data.getControlTrajectory());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.read_type_e("intervals", data.getIntervals());
      ser.read_type_e("state_trajectory", data.getStateTrajectory());
      ser.read_type_e("control_trajectory", data.getControlTrajectory());
   }

   public static void staticCopy(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage src, controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage src, controller_msgs.msg.dds.CrocoddylSolverTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CrocoddylSolverTrajectoryMessagePubSubType newInstance()
   {
      return new CrocoddylSolverTrajectoryMessagePubSubType();
   }
}

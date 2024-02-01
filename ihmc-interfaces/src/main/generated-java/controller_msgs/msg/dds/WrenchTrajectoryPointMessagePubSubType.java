package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WrenchTrajectoryPointMessage" defined in "WrenchTrajectoryPointMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WrenchTrajectoryPointMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WrenchTrajectoryPointMessage_.idl instead.
*
*/
public class WrenchTrajectoryPointMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WrenchTrajectoryPointMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WrenchTrajectoryPointMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a315dade0aa82a8cfa49789aa0247d6e3ab8ab71f917ea269f90fbb3bc99f853";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WrenchTrajectoryPointMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.WrenchPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.WrenchPubSubType.getCdrSerializedSize(data.getWrench(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getTime());

      geometry_msgs.msg.dds.WrenchPubSubType.write(data.getWrench(), cdr);
   }

   public static void read(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTime(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.WrenchPubSubType.read(data.getWrench(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("time", data.getTime());
      ser.write_type_a("wrench", new geometry_msgs.msg.dds.WrenchPubSubType(), data.getWrench());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WrenchTrajectoryPointMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTime(ser.read_type_6("time"));
      ser.read_type_a("wrench", new geometry_msgs.msg.dds.WrenchPubSubType(), data.getWrench());

   }

   public static void staticCopy(controller_msgs.msg.dds.WrenchTrajectoryPointMessage src, controller_msgs.msg.dds.WrenchTrajectoryPointMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WrenchTrajectoryPointMessage createData()
   {
      return new controller_msgs.msg.dds.WrenchTrajectoryPointMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WrenchTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WrenchTrajectoryPointMessage src, controller_msgs.msg.dds.WrenchTrajectoryPointMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WrenchTrajectoryPointMessagePubSubType newInstance()
   {
      return new WrenchTrajectoryPointMessagePubSubType();
   }
}

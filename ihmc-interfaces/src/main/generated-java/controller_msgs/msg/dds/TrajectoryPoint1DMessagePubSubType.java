package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TrajectoryPoint1DMessage" defined in "TrajectoryPoint1DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TrajectoryPoint1DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TrajectoryPoint1DMessage_.idl instead.
*
*/
public class TrajectoryPoint1DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.TrajectoryPoint1DMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::TrajectoryPoint1DMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.TrajectoryPoint1DMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TrajectoryPoint1DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.TrajectoryPoint1DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getTime());

      cdr.write_type_6(data.getPosition());

      cdr.write_type_6(data.getVelocity());

   }

   public static void read(controller_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTime(cdr.read_type_6());
      	
      data.setPosition(cdr.read_type_6());
      	
      data.setVelocity(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("time", data.getTime());
      ser.write_type_6("position", data.getPosition());
      ser.write_type_6("velocity", data.getVelocity());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.TrajectoryPoint1DMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTime(ser.read_type_6("time"));
      data.setPosition(ser.read_type_6("position"));
      data.setVelocity(ser.read_type_6("velocity"));
   }

   public static void staticCopy(controller_msgs.msg.dds.TrajectoryPoint1DMessage src, controller_msgs.msg.dds.TrajectoryPoint1DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.TrajectoryPoint1DMessage createData()
   {
      return new controller_msgs.msg.dds.TrajectoryPoint1DMessage();
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
   
   public void serialize(controller_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.TrajectoryPoint1DMessage src, controller_msgs.msg.dds.TrajectoryPoint1DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TrajectoryPoint1DMessagePubSubType newInstance()
   {
      return new TrajectoryPoint1DMessagePubSubType();
   }
}

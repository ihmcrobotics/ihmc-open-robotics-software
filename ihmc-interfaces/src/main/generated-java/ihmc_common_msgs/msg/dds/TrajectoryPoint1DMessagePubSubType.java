package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TrajectoryPoint1DMessage" defined in "TrajectoryPoint1DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TrajectoryPoint1DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TrajectoryPoint1DMessage_.idl instead.
*
*/
public class TrajectoryPoint1DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::TrajectoryPoint1DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "cde9a21a3657fa654d23bca4c6ab9cc3948cb91cbba8ecc4b1827c0d0e0ce9d3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getTime());

      cdr.write_type_6(data.getPosition());

      cdr.write_type_6(data.getVelocity());

      cdr.write_type_6(data.getAcceleration());

   }

   public static void read(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTime(cdr.read_type_6());
      	
      data.setPosition(cdr.read_type_6());
      	
      data.setVelocity(cdr.read_type_6());
      	
      data.setAcceleration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("time", data.getTime());
      ser.write_type_6("position", data.getPosition());
      ser.write_type_6("velocity", data.getVelocity());
      ser.write_type_6("acceleration", data.getAcceleration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTime(ser.read_type_6("time"));
      data.setPosition(ser.read_type_6("position"));
      data.setVelocity(ser.read_type_6("velocity"));
      data.setAcceleration(ser.read_type_6("acceleration"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage src, ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage src, ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TrajectoryPoint1DMessagePubSubType newInstance()
   {
      return new TrajectoryPoint1DMessagePubSubType();
   }
}

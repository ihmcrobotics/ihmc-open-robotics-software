package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SO3TrajectoryPointMessage" defined in "SO3TrajectoryPointMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SO3TrajectoryPointMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SO3TrajectoryPointMessage_.idl instead.
*
*/
public class SO3TrajectoryPointMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SO3TrajectoryPointMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SO3TrajectoryPointMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SO3TrajectoryPointMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SO3TrajectoryPointMessage data) throws java.io.IOException
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


      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SO3TrajectoryPointMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SO3TrajectoryPointMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAngularVelocity(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SO3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_6(data.getTime());


      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAngularVelocity(), cdr);
   }

   public static void read(controller_msgs.msg.dds.SO3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setTime(cdr.read_type_6());
      	

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAngularVelocity(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SO3TrajectoryPointMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_6("time", data.getTime());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      ser.write_type_a("angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularVelocity());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SO3TrajectoryPointMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setTime(ser.read_type_6("time"));

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());


      ser.read_type_a("angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularVelocity());

   }

   public static void staticCopy(controller_msgs.msg.dds.SO3TrajectoryPointMessage src, controller_msgs.msg.dds.SO3TrajectoryPointMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SO3TrajectoryPointMessage createData()
   {
      return new controller_msgs.msg.dds.SO3TrajectoryPointMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SO3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SO3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SO3TrajectoryPointMessage src, controller_msgs.msg.dds.SO3TrajectoryPointMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SO3TrajectoryPointMessagePubSubType newInstance()
   {
      return new SO3TrajectoryPointMessagePubSubType();
   }
}

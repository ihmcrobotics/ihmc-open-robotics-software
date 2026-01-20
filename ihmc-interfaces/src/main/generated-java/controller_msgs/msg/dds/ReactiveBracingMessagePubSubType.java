package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ReactiveBracingMessage" defined in "ReactiveBracingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ReactiveBracingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ReactiveBracingMessage_.idl instead.
*
*/
public class ReactiveBracingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ReactiveBracingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ReactiveBracingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6894783483c2d76f38953b17faa75c65e2203530687dba6eff75b701e4701db9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ReactiveBracingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ReactiveBracingMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ReactiveBracingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ReactiveBracingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getBracingPoint(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getBracingNormal(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ReactiveBracingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getBracingPoint(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getBracingNormal(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ReactiveBracingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getBracingPoint(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getBracingNormal(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ReactiveBracingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("bracing_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getBracingPoint());

      ser.write_type_a("bracing_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBracingNormal());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ReactiveBracingMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("bracing_point", new geometry_msgs.msg.dds.PointPubSubType(), data.getBracingPoint());

      ser.read_type_a("bracing_normal", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBracingNormal());

   }

   public static void staticCopy(controller_msgs.msg.dds.ReactiveBracingMessage src, controller_msgs.msg.dds.ReactiveBracingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ReactiveBracingMessage createData()
   {
      return new controller_msgs.msg.dds.ReactiveBracingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ReactiveBracingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ReactiveBracingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ReactiveBracingMessage src, controller_msgs.msg.dds.ReactiveBracingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ReactiveBracingMessagePubSubType newInstance()
   {
      return new ReactiveBracingMessagePubSubType();
   }
}

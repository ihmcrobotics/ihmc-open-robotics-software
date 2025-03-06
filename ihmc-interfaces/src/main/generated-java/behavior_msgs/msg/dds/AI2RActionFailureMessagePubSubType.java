package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AI2RActionFailureMessage" defined in "AI2RActionFailureMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AI2RActionFailureMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AI2RActionFailureMessage_.idl instead.
*
*/
public class AI2RActionFailureMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.AI2RActionFailureMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::AI2RActionFailureMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "234e04af2f9471424e0423630a28ebaf6c4d5e6fdff47030ab98012fffe3e855";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.AI2RActionFailureMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.AI2RActionFailureMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RActionFailureMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.AI2RActionFailureMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getActionName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getActionType().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getActionFrame().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPositionError(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientationError(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.AI2RActionFailureMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getActionName().length() <= 255)
      cdr.write_type_d(data.getActionName());else
          throw new RuntimeException("action_name field exceeds the maximum length: %d > %d".formatted(data.getActionName().length(), 255));

      if(data.getActionType().length() <= 255)
      cdr.write_type_d(data.getActionType());else
          throw new RuntimeException("action_type field exceeds the maximum length: %d > %d".formatted(data.getActionType().length(), 255));

      if(data.getActionFrame().length() <= 255)
      cdr.write_type_d(data.getActionFrame());else
          throw new RuntimeException("action_frame field exceeds the maximum length: %d > %d".formatted(data.getActionFrame().length(), 255));

      cdr.write_type_6(data.getPositionTolerance());

      cdr.write_type_6(data.getOrientationTolerance());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPositionError(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientationError(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.AI2RActionFailureMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getActionName());	
      cdr.read_type_d(data.getActionType());	
      cdr.read_type_d(data.getActionFrame());	
      data.setPositionTolerance(cdr.read_type_6());
      	
      data.setOrientationTolerance(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPositionError(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientationError(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.AI2RActionFailureMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("action_name", data.getActionName());
      ser.write_type_d("action_type", data.getActionType());
      ser.write_type_d("action_frame", data.getActionFrame());
      ser.write_type_6("position_tolerance", data.getPositionTolerance());
      ser.write_type_6("orientation_tolerance", data.getOrientationTolerance());
      ser.write_type_a("position_error", new geometry_msgs.msg.dds.PointPubSubType(), data.getPositionError());

      ser.write_type_a("orientation_error", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientationError());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.AI2RActionFailureMessage data)
   {
      ser.read_type_d("action_name", data.getActionName());
      ser.read_type_d("action_type", data.getActionType());
      ser.read_type_d("action_frame", data.getActionFrame());
      data.setPositionTolerance(ser.read_type_6("position_tolerance"));
      data.setOrientationTolerance(ser.read_type_6("orientation_tolerance"));
      ser.read_type_a("position_error", new geometry_msgs.msg.dds.PointPubSubType(), data.getPositionError());

      ser.read_type_a("orientation_error", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientationError());

   }

   public static void staticCopy(behavior_msgs.msg.dds.AI2RActionFailureMessage src, behavior_msgs.msg.dds.AI2RActionFailureMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.AI2RActionFailureMessage createData()
   {
      return new behavior_msgs.msg.dds.AI2RActionFailureMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.AI2RActionFailureMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.AI2RActionFailureMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.AI2RActionFailureMessage src, behavior_msgs.msg.dds.AI2RActionFailureMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AI2RActionFailureMessagePubSubType newInstance()
   {
      return new AI2RActionFailureMessagePubSubType();
   }
}

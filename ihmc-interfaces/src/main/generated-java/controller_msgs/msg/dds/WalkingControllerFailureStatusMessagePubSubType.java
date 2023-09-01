package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkingControllerFailureStatusMessage" defined in "WalkingControllerFailureStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkingControllerFailureStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkingControllerFailureStatusMessage_.idl instead.
*
*/
public class WalkingControllerFailureStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WalkingControllerFailureStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WalkingControllerFailureStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "37a7db8a1a681d1fabc807652c23be36c59439f7faca5324da3ab72e411cc7de";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getFallingDirection(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getFallingDirection(), cdr);
   }

   public static void read(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getFallingDirection(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("falling_direction", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getFallingDirection());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("falling_direction", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getFallingDirection());

   }

   public static void staticCopy(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage src, controller_msgs.msg.dds.WalkingControllerFailureStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WalkingControllerFailureStatusMessage createData()
   {
      return new controller_msgs.msg.dds.WalkingControllerFailureStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WalkingControllerFailureStatusMessage src, controller_msgs.msg.dds.WalkingControllerFailureStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkingControllerFailureStatusMessagePubSubType newInstance()
   {
      return new WalkingControllerFailureStatusMessagePubSubType();
   }
}

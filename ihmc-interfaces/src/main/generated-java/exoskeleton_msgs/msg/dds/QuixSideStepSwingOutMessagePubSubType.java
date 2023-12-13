package exoskeleton_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuixSideStepSwingOutMessage" defined in "QuixSideStepSwingOutMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuixSideStepSwingOutMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuixSideStepSwingOutMessage_.idl instead.
*
*/
public class QuixSideStepSwingOutMessagePubSubType implements us.ihmc.pubsub.TopicDataType<exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage>
{
   public static final java.lang.String name = "exoskeleton_msgs::msg::dds_::QuixSideStepSwingOutMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "137de5316121b4f404048e90339191ae10eb8db26fab27dfb83cc46b1fb4f5d5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getSideStepInSwingOut());

   }

   public static void read(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSideStepInSwingOut(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("side_step_in_swing_out", data.getSideStepInSwingOut());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data)
   {
      data.setSideStepInSwingOut(ser.read_type_7("side_step_in_swing_out"));   }

   public static void staticCopy(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage src, exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage dest)
   {
      dest.set(src);
   }

   @Override
   public exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage createData()
   {
      return new exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage();
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
   
   public void serialize(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage src, exoskeleton_msgs.msg.dds.QuixSideStepSwingOutMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuixSideStepSwingOutMessagePubSubType newInstance()
   {
      return new QuixSideStepSwingOutMessagePubSubType();
   }
}

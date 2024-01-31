package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Ramp3DMessage" defined in "Ramp3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Ramp3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Ramp3DMessage_.idl instead.
*
*/
public class Ramp3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.Ramp3DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::Ramp3DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4d1338746ea3a19b24879cea89ba8c660eaf1af338787b0f316403329ad0b3df";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.Ramp3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.Ramp3DMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Ramp3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Ramp3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getSize(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.Ramp3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getSize(), cdr);
      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.Ramp3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getSize(), cdr);	
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.Ramp3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("size", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getSize());

      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.Ramp3DMessage data)
   {
      ser.read_type_a("size", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getSize());

      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.Ramp3DMessage src, ihmc_common_msgs.msg.dds.Ramp3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.Ramp3DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.Ramp3DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.Ramp3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.Ramp3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.Ramp3DMessage src, ihmc_common_msgs.msg.dds.Ramp3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Ramp3DMessagePubSubType newInstance()
   {
      return new Ramp3DMessagePubSubType();
   }
}

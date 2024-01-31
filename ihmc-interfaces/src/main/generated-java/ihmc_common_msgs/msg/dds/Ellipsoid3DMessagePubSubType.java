package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Ellipsoid3DMessage" defined in "Ellipsoid3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Ellipsoid3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Ellipsoid3DMessage_.idl instead.
*
*/
public class Ellipsoid3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.Ellipsoid3DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::Ellipsoid3DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "01c9ae373d8d49babf49be0e47f69caa56b9a991731eb6d986a266a9bcc6b43d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPose(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRadii(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.write(data.getPose(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRadii(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PosePubSubType.read(data.getPose(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRadii(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.write_type_a("radii", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRadii());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data)
   {
      ser.read_type_a("pose", new geometry_msgs.msg.dds.PosePubSubType(), data.getPose());

      ser.read_type_a("radii", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRadii());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage src, ihmc_common_msgs.msg.dds.Ellipsoid3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.Ellipsoid3DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.Ellipsoid3DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.Ellipsoid3DMessage src, ihmc_common_msgs.msg.dds.Ellipsoid3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Ellipsoid3DMessagePubSubType newInstance()
   {
      return new Ellipsoid3DMessagePubSubType();
   }
}

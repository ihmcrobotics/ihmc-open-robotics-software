package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "Capsule3DMessage" defined in "Capsule3DMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Capsule3DMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Capsule3DMessage_.idl instead.
*
*/
public class Capsule3DMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.Capsule3DMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::Capsule3DMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5785e4784eeb54861edf180810b11e1e11b4f84be8435d387bdef9e329dcf8f4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.Capsule3DMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.Capsule3DMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Capsule3DMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.Capsule3DMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAxis(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.Capsule3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAxis(), cdr);
      cdr.write_type_6(data.getRadius());

      cdr.write_type_6(data.getLength());

   }

   public static void read(ihmc_common_msgs.msg.dds.Capsule3DMessage data, us.ihmc.idl.CDR cdr)
   {
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAxis(), cdr);	
      data.setRadius(cdr.read_type_6());
      	
      data.setLength(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.Capsule3DMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("axis", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAxis());

      ser.write_type_6("radius", data.getRadius());
      ser.write_type_6("length", data.getLength());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.Capsule3DMessage data)
   {
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("axis", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAxis());

      data.setRadius(ser.read_type_6("radius"));
      data.setLength(ser.read_type_6("length"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.Capsule3DMessage src, ihmc_common_msgs.msg.dds.Capsule3DMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.Capsule3DMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.Capsule3DMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.Capsule3DMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.Capsule3DMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.Capsule3DMessage src, ihmc_common_msgs.msg.dds.Capsule3DMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public Capsule3DMessagePubSubType newInstance()
   {
      return new Capsule3DMessagePubSubType();
   }
}

package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PolygonizerParametersStringMessage" defined in "PolygonizerParametersStringMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PolygonizerParametersStringMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PolygonizerParametersStringMessage_.idl instead.
*
*/
public class PolygonizerParametersStringMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.PolygonizerParametersStringMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::PolygonizerParametersStringMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a2d7bb77d841f2e3b1f64e128e22d4c8955998a8abd5267076474acab8f5bdd5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.PolygonizerParametersStringMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.PolygonizerParametersStringMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 2056; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PolygonizerParametersStringMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.PolygonizerParametersStringMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getParameters().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParameters().get(i0).length() + 1;
      }
      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.PolygonizerParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getParameters().size() <= 2056)
      cdr.write_type_e(data.getParameters());else
          throw new RuntimeException("parameters field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.PolygonizerParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getParameters());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.PolygonizerParametersStringMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("parameters", data.getParameters());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.PolygonizerParametersStringMessage data)
   {
      ser.read_type_e("parameters", data.getParameters());
   }

   public static void staticCopy(perception_msgs.msg.dds.PolygonizerParametersStringMessage src, perception_msgs.msg.dds.PolygonizerParametersStringMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.PolygonizerParametersStringMessage createData()
   {
      return new perception_msgs.msg.dds.PolygonizerParametersStringMessage();
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
   
   public void serialize(perception_msgs.msg.dds.PolygonizerParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.PolygonizerParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.PolygonizerParametersStringMessage src, perception_msgs.msg.dds.PolygonizerParametersStringMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PolygonizerParametersStringMessagePubSubType newInstance()
   {
      return new PolygonizerParametersStringMessagePubSubType();
   }
}

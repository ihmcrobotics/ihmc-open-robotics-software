package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConcaveHullFactoryParametersStringMessage" defined in "ConcaveHullFactoryParametersStringMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConcaveHullFactoryParametersStringMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConcaveHullFactoryParametersStringMessage_.idl instead.
*
*/
public class ConcaveHullFactoryParametersStringMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ConcaveHullFactoryParametersStringMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "92559b84e098fa36404af7476d5b968100f89723be633224040429270d2c4bec";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getParameters().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParameters().get(i0).length() + 1;
      }
      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getParameters().size() <= 2056)
      cdr.write_type_e(data.getParameters());else
          throw new RuntimeException("parameters field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getParameters());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("parameters", data.getParameters());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data)
   {
      ser.read_type_e("parameters", data.getParameters());
   }

   public static void staticCopy(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage src, controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage createData()
   {
      return new controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage src, controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConcaveHullFactoryParametersStringMessagePubSubType newInstance()
   {
      return new ConcaveHullFactoryParametersStringMessagePubSubType();
   }
}

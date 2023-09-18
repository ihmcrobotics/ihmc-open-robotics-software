package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CrocoddylFeedbackGainMessage" defined in "CrocoddylFeedbackGainMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CrocoddylFeedbackGainMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CrocoddylFeedbackGainMessage_.idl instead.
*
*/
public class CrocoddylFeedbackGainMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.CrocoddylFeedbackGainMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::CrocoddylFeedbackGainMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "edfc394538edf37178e991941b4957ed047b97f7932b8342de404c86372ce7d3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getNx());

      cdr.write_type_4(data.getNu());

      if(data.getData().size() <= 100)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setNx(cdr.read_type_4());
      	
      data.setNu(cdr.read_type_4());
      	
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("nx", data.getNx());
      ser.write_type_4("nu", data.getNu());
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data)
   {
      data.setNx(ser.read_type_4("nx"));
      data.setNu(ser.read_type_4("nu"));
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage src, controller_msgs.msg.dds.CrocoddylFeedbackGainMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.CrocoddylFeedbackGainMessage createData()
   {
      return new controller_msgs.msg.dds.CrocoddylFeedbackGainMessage();
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
   
   public void serialize(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.CrocoddylFeedbackGainMessage src, controller_msgs.msg.dds.CrocoddylFeedbackGainMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CrocoddylFeedbackGainMessagePubSubType newInstance()
   {
      return new CrocoddylFeedbackGainMessagePubSubType();
   }
}

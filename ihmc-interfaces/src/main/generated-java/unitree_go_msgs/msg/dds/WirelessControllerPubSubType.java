package unitree_go_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WirelessController" defined in "WirelessController_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WirelessController_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WirelessController_.idl instead.
*
*/
public class WirelessControllerPubSubType implements us.ihmc.pubsub.TopicDataType<unitree_go_msgs.msg.dds.WirelessController>
{
   public static final java.lang.String name = "unitree_go_msgs::msg::dds_::WirelessController_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "44c741ec748d815ecfcabd8ee94eca4160a62da7d50f1c1f94304136fbd87683";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(unitree_go_msgs.msg.dds.WirelessController data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, unitree_go_msgs.msg.dds.WirelessController data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.WirelessController data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(unitree_go_msgs.msg.dds.WirelessController data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(unitree_go_msgs.msg.dds.WirelessController data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_5(data.getLx());

      cdr.write_type_5(data.getLy());

      cdr.write_type_5(data.getRx());

      cdr.write_type_5(data.getRy());

      cdr.write_type_3(data.getKeys());

   }

   public static void read(unitree_go_msgs.msg.dds.WirelessController data, us.ihmc.idl.CDR cdr)
   {
      data.setLx(cdr.read_type_5());
      	
      data.setLy(cdr.read_type_5());
      	
      data.setRx(cdr.read_type_5());
      	
      data.setRy(cdr.read_type_5());
      	
      data.setKeys(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(unitree_go_msgs.msg.dds.WirelessController data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_5("lx", data.getLx());
      ser.write_type_5("ly", data.getLy());
      ser.write_type_5("rx", data.getRx());
      ser.write_type_5("ry", data.getRy());
      ser.write_type_3("keys", data.getKeys());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, unitree_go_msgs.msg.dds.WirelessController data)
   {
      data.setLx(ser.read_type_5("lx"));
      data.setLy(ser.read_type_5("ly"));
      data.setRx(ser.read_type_5("rx"));
      data.setRy(ser.read_type_5("ry"));
      data.setKeys(ser.read_type_3("keys"));
   }

   public static void staticCopy(unitree_go_msgs.msg.dds.WirelessController src, unitree_go_msgs.msg.dds.WirelessController dest)
   {
      dest.set(src);
   }

   @Override
   public unitree_go_msgs.msg.dds.WirelessController createData()
   {
      return new unitree_go_msgs.msg.dds.WirelessController();
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
   
   public void serialize(unitree_go_msgs.msg.dds.WirelessController data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(unitree_go_msgs.msg.dds.WirelessController data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(unitree_go_msgs.msg.dds.WirelessController src, unitree_go_msgs.msg.dds.WirelessController dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WirelessControllerPubSubType newInstance()
   {
      return new WirelessControllerPubSubType();
   }
}

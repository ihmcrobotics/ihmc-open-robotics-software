package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PIDGains" defined in "PIDGains_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PIDGains_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PIDGains_.idl instead.
*
*/
public class PIDGainsPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PIDGains>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PIDGains_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "6562b6de3dd8a12acd757a519470e6e60412ec757589c7df4c6f94afcccafc47";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PIDGains data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PIDGains data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PIDGains data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PIDGains data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PIDGains data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getKp());

      cdr.write_type_6(data.getKd());

      cdr.write_type_6(data.getKi());

      cdr.write_type_6(data.getZeta());

   }

   public static void read(controller_msgs.msg.dds.PIDGains data, us.ihmc.idl.CDR cdr)
   {
      data.setKp(cdr.read_type_6());
      	
      data.setKd(cdr.read_type_6());
      	
      data.setKi(cdr.read_type_6());
      	
      data.setZeta(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PIDGains data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("kp", data.getKp());
      ser.write_type_6("kd", data.getKd());
      ser.write_type_6("ki", data.getKi());
      ser.write_type_6("zeta", data.getZeta());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PIDGains data)
   {
      data.setKp(ser.read_type_6("kp"));
      data.setKd(ser.read_type_6("kd"));
      data.setKi(ser.read_type_6("ki"));
      data.setZeta(ser.read_type_6("zeta"));
   }

   public static void staticCopy(controller_msgs.msg.dds.PIDGains src, controller_msgs.msg.dds.PIDGains dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PIDGains createData()
   {
      return new controller_msgs.msg.dds.PIDGains();
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
   
   public void serialize(controller_msgs.msg.dds.PIDGains data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PIDGains data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PIDGains src, controller_msgs.msg.dds.PIDGains dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PIDGainsPubSubType newInstance()
   {
      return new PIDGainsPubSubType();
   }
}

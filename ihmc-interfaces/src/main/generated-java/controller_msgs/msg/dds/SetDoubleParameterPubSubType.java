package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "SetDoubleParameter" defined in "SetDoubleParameter_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from SetDoubleParameter_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit SetDoubleParameter_.idl instead.
 */
public class SetDoubleParameterPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SetDoubleParameter>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SetDoubleParameter_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public SetDoubleParameterPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SetDoubleParameter data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SetDoubleParameter data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParameterName().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SetDoubleParameter data, us.ihmc.idl.CDR cdr)
   {

      if (data.getParameterName().length() <= 255)
         cdr.write_type_d(data.getParameterName());
      else
         throw new RuntimeException("parameter_name field exceeds the maximum length");

      cdr.write_type_6(data.getParameterValue());
   }

   public static void read(controller_msgs.msg.dds.SetDoubleParameter data, us.ihmc.idl.CDR cdr)
   {

      cdr.read_type_d(data.getParameterName());

      data.setParameterValue(cdr.read_type_6());
   }

   public static void staticCopy(controller_msgs.msg.dds.SetDoubleParameter src, controller_msgs.msg.dds.SetDoubleParameter dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.SetDoubleParameter data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SetDoubleParameter data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SetDoubleParameter data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("parameter_name", data.getParameterName());

      ser.write_type_6("parameter_value", data.getParameterValue());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SetDoubleParameter data)
   {
      ser.read_type_d("parameter_name", data.getParameterName());

      data.setParameterValue(ser.read_type_6("parameter_value"));
   }

   @Override
   public controller_msgs.msg.dds.SetDoubleParameter createData()
   {
      return new controller_msgs.msg.dds.SetDoubleParameter();
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

   public void serialize(controller_msgs.msg.dds.SetDoubleParameter data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SetDoubleParameter data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.SetDoubleParameter src, controller_msgs.msg.dds.SetDoubleParameter dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SetDoubleParameterPubSubType newInstance()
   {
      return new SetDoubleParameterPubSubType();
   }
}
package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "SetStringParameter" defined in "SetStringParameter_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from SetStringParameter_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit SetStringParameter_.idl instead.
 */
public class SetStringParameterPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SetStringParameter>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SetStringParameter_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public SetStringParameterPubSubType()
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SetStringParameter data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SetStringParameter data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParameter_name().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getParameter_value().length() + 1;

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SetStringParameter data, us.ihmc.idl.CDR cdr)
   {

      if (data.getParameter_name().length() <= 255)
         cdr.write_type_d(data.getParameter_name());
      else
         throw new RuntimeException("parameter_name field exceeds the maximum length");

      if (data.getParameter_value().length() <= 255)
         cdr.write_type_d(data.getParameter_value());
      else
         throw new RuntimeException("parameter_value field exceeds the maximum length");
   }

   public static void read(controller_msgs.msg.dds.SetStringParameter data, us.ihmc.idl.CDR cdr)
   {

      cdr.read_type_d(data.getParameter_name());

      cdr.read_type_d(data.getParameter_value());
   }

   public static void staticCopy(controller_msgs.msg.dds.SetStringParameter src, controller_msgs.msg.dds.SetStringParameter dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.SetStringParameter data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SetStringParameter data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SetStringParameter data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("parameter_name", data.getParameter_name());

      ser.write_type_d("parameter_value", data.getParameter_value());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SetStringParameter data)
   {
      ser.read_type_d("parameter_name", data.getParameter_name());

      ser.read_type_d("parameter_value", data.getParameter_value());
   }

   @Override
   public controller_msgs.msg.dds.SetStringParameter createData()
   {
      return new controller_msgs.msg.dds.SetStringParameter();
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

   public void serialize(controller_msgs.msg.dds.SetStringParameter data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SetStringParameter data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.SetStringParameter src, controller_msgs.msg.dds.SetStringParameter dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SetStringParameterPubSubType newInstance()
   {
      return new SetStringParameterPubSubType();
   }
}
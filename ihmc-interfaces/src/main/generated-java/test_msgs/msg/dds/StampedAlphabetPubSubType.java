package test_msgs.msg.dds;

/**
* 
* Topic data type of the struct "StampedAlphabet" defined in "StampedAlphabet_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StampedAlphabet_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StampedAlphabet_.idl instead.
*
*/
public class StampedAlphabetPubSubType implements us.ihmc.pubsub.TopicDataType<test_msgs.msg.dds.StampedAlphabet>
{
   public static final java.lang.String name = "test_msgs::msg::dds_::StampedAlphabet_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "e1c233f99f6ac7ffddc7aec9cc66650c28744c0b82814444b76060e45c2083b8";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(test_msgs.msg.dds.StampedAlphabet data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, test_msgs.msg.dds.StampedAlphabet data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(test_msgs.msg.dds.StampedAlphabet data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(test_msgs.msg.dds.StampedAlphabet data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getLastModified(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getAlphabet().length() + 1;


      return current_alignment - initial_alignment;
   }

   public static void write(test_msgs.msg.dds.StampedAlphabet data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getLastModified(), cdr);
      if(data.getAlphabet().length() <= 255)
      cdr.write_type_d(data.getAlphabet());else
          throw new RuntimeException("alphabet field exceeds the maximum length");

   }

   public static void read(test_msgs.msg.dds.StampedAlphabet data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getLastModified(), cdr);	
      cdr.read_type_d(data.getAlphabet());	

   }

   @Override
   public final void serialize(test_msgs.msg.dds.StampedAlphabet data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("last_modified", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastModified());

      ser.write_type_d("alphabet", data.getAlphabet());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, test_msgs.msg.dds.StampedAlphabet data)
   {
      ser.read_type_a("last_modified", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLastModified());

      ser.read_type_d("alphabet", data.getAlphabet());
   }

   public static void staticCopy(test_msgs.msg.dds.StampedAlphabet src, test_msgs.msg.dds.StampedAlphabet dest)
   {
      dest.set(src);
   }

   @Override
   public test_msgs.msg.dds.StampedAlphabet createData()
   {
      return new test_msgs.msg.dds.StampedAlphabet();
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
   
   public void serialize(test_msgs.msg.dds.StampedAlphabet data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(test_msgs.msg.dds.StampedAlphabet data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(test_msgs.msg.dds.StampedAlphabet src, test_msgs.msg.dds.StampedAlphabet dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StampedAlphabetPubSubType newInstance()
   {
      return new StampedAlphabetPubSubType();
   }
}

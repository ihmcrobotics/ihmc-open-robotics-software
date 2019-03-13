package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "ReferenceFrameInformation" defined in "Handshake.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from Handshake.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit Handshake.idl instead.
*
*/
public class ReferenceFrameInformationPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.ReferenceFrameInformation>
{
   public static final java.lang.String name = "us::ihmc::robotDataLogger::ReferenceFrameInformation";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(us.ihmc.robotDataLogger.ReferenceFrameInformation data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.ReferenceFrameInformation data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (4096 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 4096; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ReferenceFrameInformation data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.ReferenceFrameInformation data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getFrameIndices().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFrameNames().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getFrameNames().get(i0).length() + 1;
      }

      return current_alignment - initial_alignment;
   }

   public static void write(us.ihmc.robotDataLogger.ReferenceFrameInformation data, us.ihmc.idl.CDR cdr)
   {
      if(data.getFrameIndices().size() <= 4096)
      cdr.write_type_e(data.getFrameIndices());else
          throw new RuntimeException("frameIndices field exceeds the maximum length");

      if(data.getFrameNames().size() <= 4096)
      cdr.write_type_e(data.getFrameNames());else
          throw new RuntimeException("frameNames field exceeds the maximum length");

   }

   public static void read(us.ihmc.robotDataLogger.ReferenceFrameInformation data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getFrameIndices());	
      cdr.read_type_e(data.getFrameNames());	

   }

   @Override
   public final void serialize(us.ihmc.robotDataLogger.ReferenceFrameInformation data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("frameIndices", data.getFrameIndices());
      ser.write_type_e("frameNames", data.getFrameNames());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.ReferenceFrameInformation data)
   {
      ser.read_type_e("frameIndices", data.getFrameIndices());
      ser.read_type_e("frameNames", data.getFrameNames());
   }

   public static void staticCopy(us.ihmc.robotDataLogger.ReferenceFrameInformation src, us.ihmc.robotDataLogger.ReferenceFrameInformation dest)
   {
      dest.set(src);
   }

   @Override
   public us.ihmc.robotDataLogger.ReferenceFrameInformation createData()
   {
      return new us.ihmc.robotDataLogger.ReferenceFrameInformation();
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
   
   public void serialize(us.ihmc.robotDataLogger.ReferenceFrameInformation data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(us.ihmc.robotDataLogger.ReferenceFrameInformation data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.ReferenceFrameInformation src, us.ihmc.robotDataLogger.ReferenceFrameInformation dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ReferenceFrameInformationPubSubType newInstance()
   {
      return new ReferenceFrameInformationPubSubType();
   }
}

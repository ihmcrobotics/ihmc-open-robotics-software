package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "Host" defined in "StaticHostList.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StaticHostList.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StaticHostList.idl instead.
*
*/
public class HostPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.Host>
{
   public static final java.lang.String name = "us::ihmc::robotDataLogger::Host";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(us.ihmc.robotDataLogger.Host data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.Host data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Host data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.Host data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getHostname().length() + 1;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);



      return current_alignment - initial_alignment;
   }

   public static void write(us.ihmc.robotDataLogger.Host data, us.ihmc.idl.CDR cdr)
   {
      if(data.getHostname().length() <= 255)
      cdr.write_type_d(data.getHostname());else
          throw new RuntimeException("hostname field exceeds the maximum length");

      cdr.write_type_3(data.getPort());

   }

   public static void read(us.ihmc.robotDataLogger.Host data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getHostname());	
      data.setPort(cdr.read_type_3());
      	

   }

   @Override
   public final void serialize(us.ihmc.robotDataLogger.Host data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("hostname", data.getHostname());
      ser.write_type_3("port", data.getPort());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.Host data)
   {
      ser.read_type_d("hostname", data.getHostname());
      data.setPort(ser.read_type_3("port"));
   }

   public static void staticCopy(us.ihmc.robotDataLogger.Host src, us.ihmc.robotDataLogger.Host dest)
   {
      dest.set(src);
   }

   @Override
   public us.ihmc.robotDataLogger.Host createData()
   {
      return new us.ihmc.robotDataLogger.Host();
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
   
   public void serialize(us.ihmc.robotDataLogger.Host data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(us.ihmc.robotDataLogger.Host data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.Host src, us.ihmc.robotDataLogger.Host dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HostPubSubType newInstance()
   {
      return new HostPubSubType();
   }
}

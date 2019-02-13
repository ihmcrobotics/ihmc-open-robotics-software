package us.ihmc.robotDataLogger;

/**
* 
* Topic data type of the struct "StaticHostList" defined in "StaticHostList.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from StaticHostList.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit StaticHostList.idl instead.
*
*/
public class StaticHostListPubSubType implements us.ihmc.pubsub.TopicDataType<us.ihmc.robotDataLogger.StaticHostList>
{
   public static final java.lang.String name = "us::ihmc::robotDataLogger::StaticHostList";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(us.ihmc.robotDataLogger.StaticHostList data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, us.ihmc.robotDataLogger.StaticHostList data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 128; ++i0)
      {
          current_alignment += us.ihmc.robotDataLogger.HostPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.StaticHostList data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(us.ihmc.robotDataLogger.StaticHostList data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getHosts().size(); ++i0)
      {
          current_alignment += us.ihmc.robotDataLogger.HostPubSubType.getCdrSerializedSize(data.getHosts().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(us.ihmc.robotDataLogger.StaticHostList data, us.ihmc.idl.CDR cdr)
   {
      if(data.getHosts().size() <= 128)
      cdr.write_type_e(data.getHosts());else
          throw new RuntimeException("hosts field exceeds the maximum length");

   }

   public static void read(us.ihmc.robotDataLogger.StaticHostList data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getHosts());	

   }

   @Override
   public final void serialize(us.ihmc.robotDataLogger.StaticHostList data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("hosts", data.getHosts());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, us.ihmc.robotDataLogger.StaticHostList data)
   {
      ser.read_type_e("hosts", data.getHosts());
   }

   public static void staticCopy(us.ihmc.robotDataLogger.StaticHostList src, us.ihmc.robotDataLogger.StaticHostList dest)
   {
      dest.set(src);
   }

   @Override
   public us.ihmc.robotDataLogger.StaticHostList createData()
   {
      return new us.ihmc.robotDataLogger.StaticHostList();
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
   
   public void serialize(us.ihmc.robotDataLogger.StaticHostList data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(us.ihmc.robotDataLogger.StaticHostList data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(us.ihmc.robotDataLogger.StaticHostList src, us.ihmc.robotDataLogger.StaticHostList dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public StaticHostListPubSubType newInstance()
   {
      return new StaticHostListPubSubType();
   }
}

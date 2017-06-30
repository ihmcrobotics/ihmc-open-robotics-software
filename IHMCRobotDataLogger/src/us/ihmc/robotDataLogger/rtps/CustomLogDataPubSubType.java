package us.ihmc.robotDataLogger.rtps;

import java.io.IOException;

import us.ihmc.idl.CDR;
import us.ihmc.idl.InterchangeSerializer;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.pubsub.common.SerializedPayload;
import us.ihmc.tools.compression.SnappyUtils;
import us.ihmc.tools.compression.SnappyUtilsTest;

/**
* 
* Topic data type of the struct "LogData" defined in "LogData.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file has been modified from the generated version to provide higher performance.
*
*/
public class CustomLogDataPubSubType implements TopicDataType<us.ihmc.robotDataLogger.LogData>
{
   public static final String name = "us::ihmc::robotDataLogger::LogData";

   private final int numberOfVariables;
   private final int numberOfStates;
   private final int maximumCompressedSize;

   public CustomLogDataPubSubType(int numberOfVariables, int numberOfStates)
   {
      this.numberOfVariables = numberOfVariables;
      this.numberOfStates = numberOfStates;
      
      maximumCompressedSize = SnappyUtils.maxCompressedLength(numberOfVariables * 8);
   }

   private final CDR serializeCDR = new CDR();
   private final CDR deserializeCDR = new CDR();

   @Override
   public void serialize(us.ihmc.robotDataLogger.LogData data, SerializedPayload serializedPayload) throws IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(SerializedPayload serializedPayload, us.ihmc.robotDataLogger.LogData data) throws IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static void write(us.ihmc.robotDataLogger.LogData data, CDR cdr)
   {

      cdr.write_type_11(data.getUid());

      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_c(data.getType().ordinal());


      cdr.write_type_2(data.getRegistry());

      if(data.getData().size() <= 100)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length");

      if(data.getJointStates().size() <= 100)
      cdr.write_type_e(data.getJointStates());else
          throw new RuntimeException("jointStates field exceeds the maximum length");

   }

   public static void read(us.ihmc.robotDataLogger.LogData data, CDR cdr)
   {


      data.setUid(cdr.read_type_11());
      

      data.setTimestamp(cdr.read_type_11());
      

      data.setType(us.ihmc.robotDataLogger.LogDataType.values[cdr.read_type_c()]);
      

      data.setRegistry(cdr.read_type_2());
      

      cdr.read_type_e(data.getData()); 

      cdr.read_type_e(data.getJointStates());
   }

   @Override
   public final void serialize(us.ihmc.robotDataLogger.LogData data, InterchangeSerializer ser)
   {
      throw new RuntimeException("Not implemented");

   }

   @Override
   public final void deserialize(InterchangeSerializer ser, us.ihmc.robotDataLogger.LogData data)
   {
      throw new RuntimeException("Not implemented");

   }

   @Override
   public us.ihmc.robotDataLogger.LogData createData()
   {
      return null;
   }

   @Override
   public int getTypeSize()
   {
      int  current_alignment = 0;
      
      current_alignment += 8 + CDR.alignment(current_alignment, 8);

      current_alignment += 8 + CDR.alignment(current_alignment, 8);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);

      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      current_alignment += (maximumCompressedSize * 1) + CDR.alignment(current_alignment, 1);


      current_alignment += 4 + CDR.alignment(current_alignment, 4);
      current_alignment += (numberOfStates * 8) + CDR.alignment(current_alignment, 8);


  
      return current_alignment;
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void serialize(us.ihmc.robotDataLogger.LogData data, CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(us.ihmc.robotDataLogger.LogData data, CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(us.ihmc.robotDataLogger.LogData src, us.ihmc.robotDataLogger.LogData dest)
   {
      throw new RuntimeException("Not implemented");
   }

   @Override
   public CustomLogDataPubSubType newInstance()
   {
      return new CustomLogDataPubSubType(numberOfVariables, numberOfStates);
   }
}
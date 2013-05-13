package us.ihmc.darpaRoboticsChallenge.networking;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import javax.media.j3d.Transform3D;

import us.ihmc.utilities.net.KryoStreamSerializer;
import us.ihmc.utilities.net.NetClassList;
import us.ihmc.utilities.net.TimestampProvider;
import us.ihmc.utilities.net.TransformableDataObject;

public class CommandRecorder
{
   final static String directory = "scripts";
   
   private final KryoStreamSerializer serializer = new KryoStreamSerializer(1048576, 1048576);
   
   private final TimestampProvider timestampProvider;
   
   private boolean isRecording = false;
   private FileOutputStream outputStream;
   private long startTime = Long.MIN_VALUE;;
   private Transform3D recordTransform = new Transform3D();
   
   public CommandRecorder(TimestampProvider timestampProvider, NetClassList netClassList)
   {
      this.timestampProvider = timestampProvider;
      serializer.registerClasses(netClassList);
      serializer.registerClass(TimestampPacket.class);
   }
   
   public synchronized void startRecording(String filename, Transform3D recordTransform)
   {
      try
      {
         File file = new File(directory + "/" + filename + ".script");
         
         if(!file.exists())
         {
            file.createNewFile();
         }
         else
         {
            throw new RuntimeException("Script " + filename + " already exists");
         }
         
         outputStream = new FileOutputStream(file, true);
         startTime = timestampProvider.getTimestamp();
         this.recordTransform.set(recordTransform);
         isRecording = true;
         System.out.println("Started recording to " + filename);
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public synchronized void stopRecording()
   {
      isRecording = false;
      try
      {
         outputStream.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      System.out.println("Stopped recording");
      startTime = Long.MIN_VALUE;
   }
   
   
   public synchronized void recordObject(Object object)
   {
      if(isRecording)
      {
         long timestamp = timestampProvider.getTimestamp() - startTime;
         TimestampPacket timestampPacket = new TimestampPacket(timestamp);
       
         
         Object objectToWrite;
         if(object instanceof TransformableDataObject)
         {
            objectToWrite = ObjectTransformationHelper.transform(recordTransform, (TransformableDataObject) object);         
         }
         else
         {
            objectToWrite = object;
         }
         
         try
         {
            serializer.write(outputStream, timestampPacket);
            serializer.write(outputStream, objectToWrite);
            outputStream.flush();
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }
   }
   
}

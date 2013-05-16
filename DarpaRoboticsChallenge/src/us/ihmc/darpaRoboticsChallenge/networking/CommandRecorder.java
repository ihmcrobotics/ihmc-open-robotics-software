package us.ihmc.darpaRoboticsChallenge.networking;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.EndOfScriptCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.HandPosePacket;
import us.ihmc.utilities.ArrayTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.KryoStreamSerializer;
import us.ihmc.utilities.net.NetClassList;
import us.ihmc.utilities.net.TimestampProvider;
import us.ihmc.utilities.net.TransformableDataObject;

public class CommandRecorder
{
   final static String directory = "scripts";
   
   private final KryoStreamSerializer serializer = new KryoStreamSerializer(1048576);
   
   private final TimestampProvider timestampProvider;
   
   private boolean isRecording = false;
   private FileOutputStream outputStream;
   private long startTime = Long.MIN_VALUE;;
   private Transform3D recordTransform = new Transform3D();

   private ArrayList<ScriptObject> scriptObjects = new ArrayList<ScriptObject>();
   private File file;


   public CommandRecorder(TimestampProvider timestampProvider, NetClassList netClassList)
   {
      this.timestampProvider = timestampProvider;
      serializer.registerClasses(netClassList);
      serializer.registerClass(TimestampPacket.class);
   }

   public synchronized void startRecording(String filename, Transform3D recordTransform)
   {
      scriptObjects.clear();

      try
      {
         file = new File(directory + "/" + filename + ".script");

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

   private synchronized void  addEndOfScriptOjectToList()
   {
      double timestamp = timestampProvider.getTimestamp() - startTime;
      EndOfScriptCommand scriptCommand = new EndOfScriptCommand();

      ScriptObject scriptObject = new ScriptObject(timestamp, scriptCommand);
      scriptObjects.add(scriptObject);
   }

   
   private synchronized void writeToFile()
   {
      for(ScriptObject scriptObject : scriptObjects)
      {
         TimestampPacket timestampPacket = new TimestampPacket((long)scriptObject.getTimeStamp());

         try
         {
            serializer.write(outputStream, timestampPacket);
            serializer.write(outputStream, scriptObject.getScriptObject());
            outputStream.flush();
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }
   }
   
   public synchronized void stopRecording()
   {
      addEndOfScriptOjectToList();
      writeToFile();

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
         double timestamp = timestampProvider.getTimestamp() - startTime;

         Object objectToWrite;
         if(object instanceof TransformableDataObject)
         {
            objectToWrite = ObjectTransformationHelper.transform(recordTransform, (TransformableDataObject) object);

            if (object instanceof HandPosePacket)
            {
               System.out.println("\n*****");
               System.out.println("CommandRecorder:transform ");
               System.out.println(recordTransform.toString());

               System.out.println("CommandRecorder: hand position before= " + ((HandPosePacket) object).position);
               FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), ((HandPosePacket) object).orientation);
               System.out.println("orienetaiton before: ");
               ArrayTools.printArray(frameOrientation.getYawPitchRoll(), System.out);


               System.out.println("CommandRecorder: hand position after= " + ((HandPosePacket) objectToWrite).position);
               frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), ((HandPosePacket) objectToWrite).orientation);
               System.out.println("orienetaiton after: ");
               ArrayTools.printArray(frameOrientation.getYawPitchRoll(), System.out);
               System.out.println("\n*****");
            }
         }
         else
         {
            objectToWrite = object;
         }

         ScriptObject scriptObject = new ScriptObject(timestamp, objectToWrite);
         scriptObjects.add(scriptObject);
      }
   }
}

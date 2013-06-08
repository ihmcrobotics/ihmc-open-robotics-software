package us.ihmc.darpaRoboticsChallenge.networking;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;

import us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.EndOfScriptCommand;
import us.ihmc.darpaRoboticsChallenge.scriptEngine.ScriptEngineSettings;
import us.ihmc.darpaRoboticsChallenge.scriptEngine.ScriptFileSaver;
import us.ihmc.utilities.FileTools;
import us.ihmc.utilities.net.NetClassList;
import us.ihmc.utilities.net.TimestampProvider;

public class CommandRecorder
{
   private final TimestampProvider timestampProvider;

   private boolean isRecording = false;
   private long startTime = Long.MIN_VALUE;;
   private Transform3D recordTransform = new Transform3D();

   private ScriptFileSaver scriptFileSaver;


   public CommandRecorder(TimestampProvider timestampProvider, NetClassList netClassList)
   {
      this.timestampProvider = timestampProvider;
   }

   public synchronized void startRecording(String filename, Transform3D recordTransform)
   {
      try
      {
         int counter = 1;
         filename = filename + ScriptEngineSettings.extension;
         boolean fileAlreadyExists = doesFileAlreadyExists(filename);

         while(fileAlreadyExists)
         {
            filename  = "duplicate"  + Integer.toString(counter) + "_";
            counter++;
            fileAlreadyExists = doesFileAlreadyExists(filename);
         }

         String path = ScriptEngineSettings.scriptDirectory + "/" + filename;


         scriptFileSaver = new ScriptFileSaver(path);
         startTime = timestampProvider.getTimestamp();
         this.recordTransform.set(recordTransform);
         isRecording = true;
         System.out.println("Started recording to " + filename);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private synchronized void addEndOfScriptOjectToList()
   {
      EndOfScriptCommand scriptCommand = new EndOfScriptCommand();
      recordObject(scriptCommand);
   }

   public synchronized void stopRecording()
   {
      addEndOfScriptOjectToList();
      isRecording = false;
      
      scriptFileSaver.close();
      
      System.out.println("Stopped recording");
      startTime = Long.MIN_VALUE;
   }


   public synchronized void recordObject(Object object)
   {
      if(isRecording)
      {
         long timestamp = timestampProvider.getTimestamp() - startTime;

         try
         {
            scriptFileSaver.recordObject(timestamp, object, recordTransform);
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }
   }

   private boolean doesFileAlreadyExists(String filename)
   {
      filename = filename + ScriptEngineSettings.extension;
      File directory = new File(ScriptEngineSettings.scriptDirectory);

      ArrayList<File> files = FileTools.getAllFilesInDirectoryRecursive(directory);

      for (File file : files)
      {
         if (file.getName().equals(filename))
            return true;
      }

      return false;
   }
}

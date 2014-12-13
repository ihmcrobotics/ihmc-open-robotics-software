package us.ihmc.darpaRoboticsChallenge.networking;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.communication.packets.walking.EndOfScriptCommand;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptEngineSettings;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileSaver;
import us.ihmc.utilities.FileTools;
import us.ihmc.utilities.TimestampProvider;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class CommandRecorder
{
   private final TimestampProvider timestampProvider;

   private boolean isRecording = false;
   private long startTime = Long.MIN_VALUE;
   private ReferenceFrame recordFrame;
   private RigidBodyTransform recordTransform = new RigidBodyTransform();

   private ScriptFileSaver scriptFileSaver;


   public CommandRecorder(TimestampProvider timestampProvider)
   {
      this.timestampProvider = timestampProvider;
   }


   public synchronized void startRecording(String originalFilename, ReferenceFrame recordFrame)
   {
      try
      {
         int counter = 1;
         originalFilename = originalFilename + ScriptEngineSettings.extension;
         String proposedFilename = new String(originalFilename);
         boolean fileAlreadyExists = doesFileAlreadyExists(proposedFilename);

         while(fileAlreadyExists)
         {
            proposedFilename  = "duplicate"  + Integer.toString(counter) + "_" + originalFilename;
            counter++;
            fileAlreadyExists = doesFileAlreadyExists(proposedFilename);
         }

         String path = ScriptEngineSettings.scriptSavingDirectory + "/" + proposedFilename;


         scriptFileSaver = new ScriptFileSaver(path, false);
         startTime = timestampProvider.getTimestamp();
         this.recordFrame = recordFrame;
         isRecording = true;
         System.out.println("CommandRecorder: Started recording to " + proposedFilename);
      }
      catch (IOException e)
      {
         System.out.println("CommandRecorder: fileanme =" + originalFilename);
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

      try
      {
         scriptFileSaver.close();
      }
      catch (Exception e1)
      {
         System.err.print("CommandRecorder: Failed stop recording.");
      }
      
      System.out.println("CommandRecorder: Stopped recording");
      startTime = Long.MIN_VALUE;
   }


   public synchronized void recordObject(Object object)
   {
      if(isRecording)
      {
         long timestamp = timestampProvider.getTimestamp() - startTime;

         try
         {
            
            if(recordFrame!=null)
            {
               recordFrame.update();
               recordTransform = new RigidBodyTransform(ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(recordFrame));
               
            }
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
      File directory = new File(ScriptEngineSettings.scriptSavingDirectory);

      ArrayList<File> files = FileTools.getAllFilesInDirectoryRecursive(directory);

      for (File file : files)
      {
         if (file.getName().equals(filename))
            return true;
      }

      return false;
   }
}

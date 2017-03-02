package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Path;
import java.util.ArrayList;

import com.thoughtworks.xstream.io.StreamException;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.io.printing.PrintTools;

public class ScriptEngine
{
   private final ScriptEngineUIInterface controllerHandler;

   private boolean isRecording = false;
   private ReferenceFrame scriptFrame;
   private RigidBodyTransform playbackTransform = new RigidBodyTransform();

   private File lastScriptFileLoaded = null;

   public ScriptEngine(ScriptEngineUIInterface controllerHandler)
   {
      this.controllerHandler = controllerHandler;
   }

   public void startRecording(Path scriptDirectory, String baseFileName)
   {
      this.controllerHandler.startRecordingScript(scriptDirectory, baseFileName, scriptFrame);
      isRecording = true;
   }

   public void stopRecording()
   {
      isRecording = false;
      this.controllerHandler.stopRecordingScript();
   }

   public void setFramesForRecord(ReferenceFrame currentScriptFrame)
   {
      this.scriptFrame = currentScriptFrame;

      //    System.out.println("ScriptEngine: scriptFrameTransform= \n" + scriptFrameTransform.toString());
   }

   public boolean isRecording()
   {
      return isRecording;
   }

   public ArrayList<ScriptObject> getLastLoadedScriptFile()
   {
      if (lastScriptFileLoaded == null)
         return null;

      return getScriptObjectsFromFile(lastScriptFileLoaded);
   }

   public ArrayList<ScriptObject> getScriptObjectsFromFile(File file)
   {
      ScriptFileLoader loader;
      try
      {
         loader = new ScriptFileLoader(file);
         ArrayList<ScriptObject> scriptObjects = loader.readIntoList();
         loader.close();

         return scriptObjects;
      }
      catch (IOException | StreamException exception)
      {
         PrintTools.error("Problem loading file. Try another one. Sorry :(.");
         return null;
      }
   }
   
   public ArrayList<ScriptObject> getScriptObjectsFromInputStream(InputStream inStream)
   {
      ScriptFileLoader loader;
      try
      {
         loader = new ScriptFileLoader(inStream);
         ArrayList<ScriptObject> scriptObjects = loader.readIntoList();
         loader.close();

         return scriptObjects;
      }
      catch (IOException e)
      {
         System.err.println("Problem loading file. Try another one. Sorry :(.");

         return null;
      }
   }

   public ArrayList<ScriptObject> getScriptObjects()
   {
      File file = ScriptFileSelector.getScriptFileFromUserSelection(ScriptEngineSettings.extension);

      return getScriptObjects(file);
   }

   public ArrayList<ScriptObject> getScriptObjects(File file)
   {
      if (file != null)
      {
         lastScriptFileLoaded = file;

         return getScriptObjectsFromFile(file);
      }
      else
         return null;
   }
   
   public ArrayList<ScriptObject> getScriptObjects(InputStream inStream)
   {
      if (inStream != null)
      {
         return getScriptObjectsFromInputStream(inStream);
      }
      else
         return null;
   }

   public String getLastScriptFileLoaded()
   {
      return lastScriptFileLoaded.getName();
   }
}

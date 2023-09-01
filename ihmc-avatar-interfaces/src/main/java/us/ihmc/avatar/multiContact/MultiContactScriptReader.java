package us.ihmc.avatar.multiContact;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.log.LogTools;

public class MultiContactScriptReader
{
   private int currentMessageIndex = 0;
   private final List<FrameShape3DBasics> loadedEnvironmentShapes = new ArrayList<>();
   private final List<KinematicsToolboxSnapshotDescription> loadedScriptKeyFrames = new ArrayList<>();

   public MultiContactScriptReader()
   {
   }

   public boolean loadScript(File scriptFile)
   {
      if (scriptFile == null)
      {
         LogTools.info("No file provided");
         return false;
      }

      if (!scriptFile.exists())
      {
         LogTools.info("Script file does not exist: " + scriptFile);
         return false;
      }

      if (!scriptFile.isFile())
      {
         LogTools.info("Not a file: " + scriptFile);
         return false;
      }

      try
      {
         return loadScript(new FileInputStream(scriptFile));
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   public boolean loadScript(InputStream inputStream)
   {
      if (inputStream == null)
      {
         LogTools.info("Stream is null");
         return false;
      }

      try
      {
         ObjectMapper objectMapper = new ObjectMapper();
         JsonNode jsonNode = objectMapper.readTree(inputStream);

         JsonNode environmentNode = jsonNode.get(MultiContactEnvironmentDescription.ENVIRONMENT_JSON);
         JsonNode scriptNode = jsonNode.get(KinematicsToolboxSnapshotDescription.SCRIPT_JSON);

         if (environmentNode == null || scriptNode == null)
            return false;

         List<FrameShape3DBasics> environmentShapes = new ArrayList<>();
         List<KinematicsToolboxSnapshotDescription> scriptKeyFrames = new ArrayList<>();

         for (int i = 0; i < environmentNode.size(); i++)
         {
            ObjectNode environmentShape = (ObjectNode) environmentNode.get(i);
            environmentShapes.add(MultiContactEnvironmentDescription.fromJSON(environmentShape));
         }

         loadedEnvironmentShapes.clear();
         loadedEnvironmentShapes.addAll(environmentShapes);

         for (int i = 0; i < scriptNode.size(); i++)
         {
            JsonNode keyframe = scriptNode.get(i);
            scriptKeyFrames.add(KinematicsToolboxSnapshotDescription.fromJSON(keyframe));
         }

         loadedScriptKeyFrames.clear();
         loadedScriptKeyFrames.addAll(scriptKeyFrames);

         currentMessageIndex = -1;
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   public int size()
   {
      return loadedScriptKeyFrames.size();
   }

   public int getCurrentMessageIndex()
   {
      return currentMessageIndex;
   }

   public boolean hasNext()
   {
      return currentMessageIndex < loadedScriptKeyFrames.size() - 1;
   }

   public KinematicsToolboxSnapshotDescription rewind()
   {
      currentMessageIndex = 0;
      return getCurrent();
   }

   public KinematicsToolboxSnapshotDescription getFirst()
   {
      if (loadedScriptKeyFrames.isEmpty())
         return null;
      else
         return loadedScriptKeyFrames.get(0);
   }

   public KinematicsToolboxSnapshotDescription next()
   {
      currentMessageIndex++;
      if (currentMessageIndex >= loadedScriptKeyFrames.size())
         currentMessageIndex = loadedScriptKeyFrames.size() - 1;
      return getCurrent();
   }

   public KinematicsToolboxSnapshotDescription previous()
   {
      currentMessageIndex--;
      if (currentMessageIndex < 0)
         currentMessageIndex = 0;
      return getCurrent();
   }

   public KinematicsToolboxSnapshotDescription getCurrent()
   {
      if (loadedScriptKeyFrames.isEmpty() || currentMessageIndex == -1)
         return null;
      else
         return loadedScriptKeyFrames.get(currentMessageIndex);
   }

   public List<FrameShape3DBasics> getEnvironmentShapes()
   {
      return loadedEnvironmentShapes;
   }

   public List<KinematicsToolboxSnapshotDescription> getAllItems()
   {
      return loadedScriptKeyFrames;
   }

   public void applyTransform(Transform transform)
   {
      for (KinematicsToolboxSnapshotDescription description : loadedScriptKeyFrames)
      {
         description.applyTransform(transform);
      }
   }
}

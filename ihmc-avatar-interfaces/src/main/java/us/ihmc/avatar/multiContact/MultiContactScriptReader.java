package us.ihmc.avatar.multiContact;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.log.LogTools;

public class MultiContactScriptReader
{
   private int currentMessageIndex = 0;
   private final List<KinematicsToolboxSnapshotDescription> loadedMessages = new ArrayList<>();

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
         FileInputStream fileInputStream = new FileInputStream(scriptFile);
         ObjectMapper objectMapper = new ObjectMapper();
         JsonNode jsonNode = objectMapper.readTree(fileInputStream);

         List<KinematicsToolboxSnapshotDescription> messages = new ArrayList<>();

         for (int i = 0; i < jsonNode.size(); i++)
         {
            JsonNode child = jsonNode.get(i);
            messages.add(KinematicsToolboxSnapshotDescription.fromJSON(child));
         }

         loadedMessages.clear();
         loadedMessages.addAll(messages);
         currentMessageIndex = -1;
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   public boolean hasNext()
   {
      return currentMessageIndex < loadedMessages.size() - 1;
   }

   public KinematicsToolboxSnapshotDescription rewind()
   {
      currentMessageIndex = 0;
      return getCurrent();
   }

   public KinematicsToolboxSnapshotDescription getFirst()
   {
      if (loadedMessages.isEmpty())
         return null;
      else
         return loadedMessages.get(0);
   }

   public KinematicsToolboxSnapshotDescription next()
   {
      currentMessageIndex++;
      if (currentMessageIndex >= loadedMessages.size())
         currentMessageIndex = loadedMessages.size() - 1;
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
      if (loadedMessages.isEmpty() || currentMessageIndex == -1)
         return null;
      else
         return loadedMessages.get(currentMessageIndex);
   }

   public void applyTransform(Transform transform)
   {
      for (KinematicsToolboxSnapshotDescription description : loadedMessages)
      {
         description.applyTransform(transform);
      }
   }
}

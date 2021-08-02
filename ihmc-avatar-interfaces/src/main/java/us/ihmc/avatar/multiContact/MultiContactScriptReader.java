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

   public int size()
   {
      return loadedMessages.size();
   }

   public int getCurrentMessageIndex()
   {
      return currentMessageIndex;
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

   public List<KinematicsToolboxSnapshotDescription> getAllItems()
   {
      return loadedMessages;
   }

   public void applyTransform(Transform transform)
   {
      for (KinematicsToolboxSnapshotDescription description : loadedMessages)
      {
         description.applyTransform(transform);
      }
   }
}

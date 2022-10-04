package us.ihmc.avatar.multiContact;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;

import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.nio.FileTools;

public class MultiContactScriptWriter
{
   private File scriptFile = null;
   private final List<KinematicsToolboxSnapshotDescription> messagesToWrite = new ArrayList<>();

   public MultiContactScriptWriter()
   {
   }

   public boolean startNewScript(File scriptFile, boolean overrideExistingFile)
   {
      if (!scriptFile.exists())
      {
         try
         {
            FileTools.ensureDirectoryExists(scriptFile.getParentFile().toPath());
            scriptFile.createNewFile();
         }
         catch (IOException e)
         {
            e.printStackTrace();
            return false;
         }
      }
      else
      {
         if (!overrideExistingFile)
            return false;

         try
         {
            scriptFile.delete();
            scriptFile.createNewFile();
         }
         catch (IOException e)
         {
            e.printStackTrace();
            return false;
         }
      }

      this.scriptFile = scriptFile;

      return true;
   }

   public void clear()
   {
      messagesToWrite.clear();
   }

   public void recordConfiguration(KinematicsToolboxSnapshotDescription description)
   {
      messagesToWrite.add(description);
   }

   public int getCurrentScriptSize()
   {
      return messagesToWrite.size();
   }

   public KinematicsToolboxSnapshotDescription getKeyFrame(int index)
   {
      return messagesToWrite.get(index);
   }

   public void remove(int index)
   {
      messagesToWrite.remove(index);
   }

   public boolean isEmpty()
   {
      return messagesToWrite.isEmpty();
   }

   public List<KinematicsToolboxSnapshotDescription> getAllItems()
   {
      return messagesToWrite;
   }

   public boolean removeLast()
   {
      if (messagesToWrite.isEmpty())
         return false;
      messagesToWrite.remove(messagesToWrite.size() - 1);
      return true;
   }

   public boolean writeScript()
   {
      PrintStream printStream = null;

      try
      {
         printStream = new PrintStream(scriptFile);
         JsonFactory jsonFactory = new JsonFactory();
         ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
         ArrayNode arrayNode = objectMapper.createArrayNode();

         for (KinematicsToolboxSnapshotDescription message : messagesToWrite)
            arrayNode.add(message.toJSON(objectMapper));

         objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, arrayNode);
         printStream.close();
         scriptFile = null;
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         if (printStream != null)
            printStream.close();
         return false;
      }
   }

   public boolean writeScriptNew()
   {
      PrintStream printStream = null;

      try
      {
         printStream = new PrintStream(scriptFile);
         JsonFactory jsonFactory = new JsonFactory();
         ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
         ObjectNode rootNode = objectMapper.createObjectNode();

         ObjectNode environmentNode = rootNode.putObject(KinematicsToolboxSnapshotDescription.ENVIRONMENT_JSON);
         ArrayNode scriptNode = rootNode.putArray(KinematicsToolboxSnapshotDescription.SCRIPT_JSON);

         for (KinematicsToolboxSnapshotDescription message : messagesToWrite)
            scriptNode.add(message.toJSON(objectMapper));

         objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, rootNode);
         printStream.close();
         scriptFile = null;
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         if (printStream != null)
            printStream.close();
         return false;
      }
   }
}

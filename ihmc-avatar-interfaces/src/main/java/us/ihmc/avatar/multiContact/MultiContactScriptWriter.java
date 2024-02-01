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
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;

public class MultiContactScriptWriter
{
   private File scriptFile = null;

   private final List<FrameShape3DReadOnly> environmentShapes = new ArrayList<>();
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

   public void clearEnvironment()
   {
      environmentShapes.clear();
   }

   public void addEnvironmentShape(FrameShape3DReadOnly environmentShape)
   {
      this.environmentShapes.add(environmentShape);
   }

   public void setEnvironmentShapes(List<FrameShape3DBasics> environmentShapes)
   {
      this.environmentShapes.clear();
      this.environmentShapes.addAll(environmentShapes);
   }

   public void clearScript()
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
         ObjectNode rootNode = objectMapper.createObjectNode();

         ArrayNode environmentNode = rootNode.putArray(MultiContactEnvironmentDescription.ENVIRONMENT_JSON);
         ArrayNode scriptNode = rootNode.putArray(KinematicsToolboxSnapshotDescription.SCRIPT_JSON);

         for (FrameShape3DReadOnly environmentShape : environmentShapes)
            environmentNode.add(MultiContactEnvironmentDescription.toJSON(environmentShape));

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

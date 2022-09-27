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
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

import static us.ihmc.avatar.multiContact.MultiContactScriptReader.ENVIRONMENT_TAG;
import static us.ihmc.avatar.multiContact.MultiContactScriptReader.SCRIPT_TAG;

public class MultiContactScriptWriter
{
   private File scriptFile = null;
   private final List<KinematicsToolboxSnapshotDescription> messagesToWrite = new ArrayList<>();
   private final List<FrameShape3DReadOnly> environmentShapes = new ArrayList<>();

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

   public void setEnvironmentShapes(List<FrameShape3DReadOnly> environmentShapes)
   {
      this.environmentShapes.addAll(environmentShapes);
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

         ObjectNode environment = rootNode.putObject(ENVIRONMENT_TAG);
         ArrayNode script = rootNode.putArray(SCRIPT_TAG);

         MultiContactEnvironmentIOTools.writeToJSON(objectMapper, environment, environmentShapes);

         for (KinematicsToolboxSnapshotDescription message : messagesToWrite)
            script.add(message.toJSON(objectMapper));

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

   public static void main(String[] args)
   {
      MultiContactScriptWriter scriptWriter = new MultiContactScriptWriter();

      File scriptFile = new File(System.getProperty("user.home") + File.separator + "TmpScriptFile.json");

      List<FrameShape3DReadOnly> shapes = new ArrayList<>();
      FrameBox3D boxA = new FrameBox3D(ReferenceFrame.getWorldFrame(), new Point3D(2.0, 0.0, 0.0), new Quaternion(0.0, 0.3, 0.0), 0.5, 0.5, 0.1);
      shapes.add(boxA);
      FrameBox3D boxB = new FrameBox3D(ReferenceFrame.getWorldFrame(), new Point3D(2.5, 0.0, 0.1), new Quaternion(0.0, 0.3, 0.1), 0.1, 0.1, 0.3);
      shapes.add(boxB);
      scriptWriter.setEnvironmentShapes(shapes);

      scriptWriter.startNewScript(scriptFile, true);

      KinematicsToolboxSnapshotDescription snapshot = new KinematicsToolboxSnapshotDescription();
      snapshot.setControllerConfiguration(new RobotConfigurationData());
      snapshot.setIkSolution(new KinematicsToolboxOutputStatus());
      snapshot.setIkPrivilegedConfiguration(new KinematicsToolboxPrivilegedConfigurationMessage());
      snapshot.setExecutionDuration(2.0);
      snapshot.setSixDoFAnchors(new ArrayList<>());
      snapshot.setOneDoFAnchors(new ArrayList<>());

      scriptWriter.recordConfiguration(snapshot);
      scriptWriter.writeScript();
   }
}

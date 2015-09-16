package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javax.vecmath.Vector3d;

import com.thoughtworks.xstream.converters.ConversionException;
import com.thoughtworks.xstream.io.StreamException;
import com.thoughtworks.xstream.mapper.CannotResolveClassException;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class ScriptTransformer
{
   private static final String ORIGINAL = "Original";

   private static final Runtime rt = Runtime.getRuntime();

   private final List<String> namesOfFilesToTransform = new ArrayList<>();
   private final HashSet<String> listOfDirectoriesContainingAtLeastOneScript = new HashSet<>();
   private final List<String> listOfDirectoriesToCreate = new ArrayList<String>();

   private final RigidBodyTransform identity = new RigidBodyTransform();

   public ScriptTransformer(String scriptDirectoryPath) throws IOException, InterruptedException
   {
      findScripts(scriptDirectoryPath, namesOfFilesToTransform, listOfDirectoriesContainingAtLeastOneScript);

      for (String directory : listOfDirectoriesContainingAtLeastOneScript)
      {
         String directoryWithTransformedScripts = directory.replaceFirst(ORIGINAL, "");
         listOfDirectoriesToCreate.add(directoryWithTransformedScripts);
         rt.exec(new String[] { "cmd", "/C", "mkdir " + directoryWithTransformedScripts });
      }

      ThreadTools.sleepSeconds(1.0);

      for (String script : namesOfFilesToTransform)
      {
         String newScript = script.replaceFirst(ORIGINAL, "");
         ArrayList<ScriptObject> scriptObjects = new ArrayList<>();
         ScriptFileLoader scriptFileLoader = null;
         try
         {
            scriptFileLoader = new ScriptFileLoader(script);
         }
         catch (StreamException e)
         {
            PrintTools.error("Empty script: " + script.substring(script.lastIndexOf("\\") + 1));
            continue;
         }

         while (true)
         {
            try
            {
               scriptObjects.add(scriptFileLoader.getScriptObject(identity));
            }
            catch (ConversionException | CannotResolveClassException e)
            {
               if (e.getCause() != null)
                  PrintTools.error("In script: " + script.substring(script.lastIndexOf("\\") + 1) + ". Problem loading packet, cause: " + e.getCause().getMessage());
               else
                  PrintTools.error("In script: " + script.substring(script.lastIndexOf("\\") + 1) + ". Problem loading packet, exception: " + e.getClass().getSimpleName() + ", message: " + e.getMessage());
               continue;
            }
            catch (IOException | StreamException e)
            {
               break;
            }
         }
         scriptFileLoader.close();

         for (ScriptObject scriptObject : scriptObjects)
         {
            Object object = scriptObject.getScriptObject();

            if (object instanceof HandPosePacket)
            {
               HandPosePacket packetToTransform = (HandPosePacket) object;
               if (packetToTransform.orientation == null || packetToTransform.position == null)
                  continue;
               RigidBodyTransform handpose = new RigidBodyTransform();
               handpose.setRotation(packetToTransform.getOrientation());
               handpose.setTranslation(new Vector3d(packetToTransform.position));
               RigidBodyTransform xOffsetInHandFrame = new RigidBodyTransform();
               xOffsetInHandFrame.setTranslation(0.16, 0.0, 0.0);
               handpose.multiply(handpose, xOffsetInHandFrame);
               handpose.getTranslation(packetToTransform.position);
            }
         }

         ScriptFileSaver scriptFileSaver = null;
         try
         {
            scriptFileSaver = new ScriptFileSaver(newScript, true);
         }
         catch (IOException e)
         {
            PrintTools.error("During writing: " + e.getClass().getSimpleName() + " for the file:" + newScript);
            if (e.getCause() != null)
               PrintTools.error(e.getCause().getMessage());
            else
               PrintTools.error(e.getMessage());
            continue;
         }

         for (ScriptObject scriptObject : scriptObjects)
            scriptFileSaver.recordObject(scriptObject.getTimeStamp(), scriptObject.getScriptObject());
         
         scriptFileSaver.close();
         System.out.println("Done transforming script: " + newScript);
      }

      Process exec = rt.exec(new String[] { "cmd", "/C", "rmdir /Q /S " + scriptDirectoryPath});
      ThreadTools.sleepSeconds(0.1);
   }

   private static void findScripts(final String scriptDirectoryPath, final List<String> fileNamesToPack,
         final Set<String> listOfDirectoriesContainingAtLeastOneScript) throws IOException
   {
      final List<String> potentialSubDirectories = new ArrayList<String>();
      final Process proc = rt.exec(new String[] { "cmd", "/C", "dir /B " + scriptDirectoryPath });

      Runnable settingsCreator = new Runnable()
      {
         @Override
         public void run()
         {
            String temp = null;
            InputStreamReader isr = new InputStreamReader(proc.getInputStream());
            BufferedReader br = new BufferedReader(isr);
            try
            {
               while ((temp = br.readLine()) != null)
               {
                  String fileName = scriptDirectoryPath + "\\" + temp;
                  if (temp.endsWith(".xml"))
                  {
                     listOfDirectoriesContainingAtLeastOneScript.add(scriptDirectoryPath);
                     fileNamesToPack.add(fileName);
                  }
                  else
                     potentialSubDirectories.add(fileName);
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      };
      new Thread(settingsCreator).start();

      ThreadTools.sleepSeconds(0.1);

      for (int i = 0; i < potentialSubDirectories.size(); i++)
      {
         findScripts(potentialSubDirectories.get(i), fileNamesToPack, listOfDirectoriesContainingAtLeastOneScript);
      }
   }

   public Iterable<String> getFileNames()
   {
      return namesOfFilesToTransform;
   }

   public Iterable<String> getListOfOriginalDirectories()
   {
      return listOfDirectoriesContainingAtLeastOneScript;
   }

   public Iterable<String> getListOfNewDirectories()
   {
      return listOfDirectoriesToCreate;
   }

   public static void main(String[] args) throws IOException, InterruptedException
   {
      ArrayList<String> paths = new ArrayList<>();
      paths.add("..\\Atlas\\scripts");
      paths.add("..\\DarpaRoboticsChallenge\\scripts");
      paths.add("..\\DarpaRoboticsChallenge\\scriptsSaved");
      paths.add("..\\DarpaRoboticsChallenge\\resources\\scripts\\ExerciseAndJUnitScripts");
      paths.add("..\\IHMCHumanoidBehaviors\\scripts");
      paths.add("..\\IHMCHumanoidBehaviors\\\resources\\scripts");
      paths.add("..\\IHMCHumanoidOperatorInterface\\scripts");
      paths.add("..\\IHMCHumanoidOperatorInterface\\resources\\finalScripts");

      ArrayList<String> newPaths = moveScriptDirectories(paths, ORIGINAL);

      int index = 0;
      for (String path : newPaths)
      {
         System.out.println("Transforming scripts in: " + paths.get(index++));
         new ScriptTransformer(path);
      }
   }

   private static ArrayList<String> moveScriptDirectories(ArrayList<String> paths, String original) throws IOException
   {
      ArrayList<String> newPaths = new ArrayList<>();
      for (String originalPath : paths)
      {
         rt.exec(new String[] { "cmd", "/C", "move " + originalPath + " " + originalPath + original });
         ThreadTools.sleepSeconds(0.1);
         newPaths.add(originalPath + original);
      }
      return newPaths;
   }
}

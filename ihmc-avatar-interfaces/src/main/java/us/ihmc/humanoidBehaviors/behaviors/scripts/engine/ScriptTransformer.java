package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.thoughtworks.xstream.converters.ConversionException;
import com.thoughtworks.xstream.io.StreamException;
import com.thoughtworks.xstream.mapper.CannotResolveClassException;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public abstract class ScriptTransformer
{
   private final List<File> filesToTransform = new ArrayList<>();
   private final HashSet<String> listOfDirectoriesContainingAtLeastOneScript = new HashSet<>();
   private final List<String> listOfDirectoriesToCreate = new ArrayList<String>();

   private final RigidBodyTransform identity = new RigidBodyTransform();

   public ScriptTransformer(String scriptDirectoryPath) throws IOException, InterruptedException
   {
      findScripts(scriptDirectoryPath, filesToTransform, listOfDirectoriesContainingAtLeastOneScript);

      for (File scriptFileToTransform : filesToTransform)
      {
    	  if (scriptFileToTransform.isDirectory())
    	  {
    		  System.err.println("File should not be a director!?");
    		  continue;
    	  }

    	  File parentDirectory = scriptFileToTransform.getParentFile();
    	  if (parentDirectory.isDirectory())
    	  {
    	     File subDirectoryToMoveOriginals = new File(parentDirectory, "OriginalBackups");
    	     if (!subDirectoryToMoveOriginals.exists()) subDirectoryToMoveOriginals.mkdir();

    	     File scriptFileToTransformBackup = new File(subDirectoryToMoveOriginals, scriptFileToTransform.getName());
    	     Files.copy(scriptFileToTransform.toPath(), scriptFileToTransformBackup.toPath(), StandardCopyOption.REPLACE_EXISTING);
    	  }

         ArrayList<ScriptObject> scriptObjects = new ArrayList<>();
         ArrayList<Object> newScriptObjects = new ArrayList<>();
         ScriptFileLoader scriptFileLoader = null;

         String scriptName = scriptFileToTransform.getName();

         try
         {
            scriptFileLoader = new ScriptFileLoader(scriptFileToTransform);
         }
         catch (StreamException e)
         {
            PrintTools.error("Empty script: " + scriptName);
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
                  PrintTools.error("In script: " + scriptName + ". Problem loading packet, cause: " + e.getCause().getMessage());
               else
                  PrintTools.error("In script: " + scriptName + ". Problem loading packet, exception: " + e.getClass().getSimpleName() + ", message: " + e.getMessage());
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
            Object newScriptObject = transformScriptObject(object);
            newScriptObjects.add(newScriptObject);
         }

         File newScript = new File(scriptFileToTransform.getParentFile(), scriptFileToTransform.getName());
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

         for (Object newObject : newScriptObjects)
         {
            ScriptObject newScriptObject = new ScriptObject(System.currentTimeMillis(), newObject);
            scriptFileSaver.recordObject(newScriptObject.getTimeStamp(), newScriptObject.getScriptObject());
         }

         scriptFileSaver.close();
         System.out.println("Done transforming script: " + newScript);
      }
   }

   public abstract Object transformScriptObject(Object object);

   private static void findScripts(final String scriptDirectoryPath, final List<File> foundScriptFilesToPack,
		   final Set<String> listOfDirectoriesContainingAtLeastOneScript) throws IOException
   {
	   final List<String> potentialSubDirectories = new ArrayList<String>();

	   File directory = new File(scriptDirectoryPath);
	   if (!directory.isDirectory())
	   {
	      if (directory.getName().endsWith(".xml"))
         {
            System.out.println("Found script file: " + directory.getName());
            foundScriptFilesToPack.add(directory);
         }
		   return;
	   }

	   System.out.println(directory.getAbsolutePath() + " is a directory. Exploring contents");

	   File[] filesInDirectory = directory.listFiles();

	   for (File file : filesInDirectory)
	   {
		   String filename = file.getName();
		   if (filename.endsWith(".xml"))
		   {
			   System.out.println("Found script file: " + filename);
			   foundScriptFilesToPack.add(file);
		   }
	   }

	   for (int i = 0; i < potentialSubDirectories.size(); i++)
	   {
		   findScripts(potentialSubDirectories.get(i), foundScriptFilesToPack, listOfDirectoriesContainingAtLeastOneScript);
	   }
   }

   public Iterable<File> getFilesToTransform()
   {
      return filesToTransform;
   }

   public Iterable<String> getListOfOriginalDirectories()
   {
      return listOfDirectoriesContainingAtLeastOneScript;
   }

   public Iterable<String> getListOfNewDirectories()
   {
      return listOfDirectoriesToCreate;
   }

}

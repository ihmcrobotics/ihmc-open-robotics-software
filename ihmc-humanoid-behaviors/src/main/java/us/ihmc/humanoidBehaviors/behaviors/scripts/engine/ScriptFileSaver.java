package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.nio.file.Path;
import java.util.ArrayList;

import com.thoughtworks.xstream.XStream;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ScriptFileSaver
{
   private final FileWriter fileWriter;
   private final ObjectOutputStream outputStream;

   public ScriptFileSaver(Path scriptFilePath, boolean overwriteExistingScript) throws IOException
   {
      this(scriptFilePath.toFile(), overwriteExistingScript);
   }
   
   public ScriptFileSaver(File file, boolean overwriteExistingScript) throws IOException
   {
      XStream xStream = new XStream();
      if (!file.exists() || overwriteExistingScript)
      {
         file.createNewFile();
      }
      else
      {
         throw new IOException("File already exists");
      }

      fileWriter = new FileWriter(file.getAbsoluteFile());
      outputStream = xStream.createObjectOutputStream(fileWriter);
   }

   /**
    * @deprecated This method is highly inefficient. The whole scripting needs to be redone to better implement the transform.
    */
   public void recordObject(long timestamp, Object object, RigidBodyTransform objectTransform) throws IOException
   {

      Object objectToWrite;

      try
      {
         objectToWrite = object.getClass().getConstructor(object.getClass()).newInstance(object);
      }
      catch (Exception e)
      {
         try
         {
            objectToWrite = object.getClass().getConstructor().newInstance();
            objectToWrite.getClass().getMethod("set", object.getClass()).invoke(objectToWrite, object);
         }
         catch (Exception e1)
         {
            String className = object.getClass().getSimpleName();
            PrintTools.error("The class: " + className + " needs either a copy constructor: " + className + "(" + className + "), or a setter: " + className + "set(" + className + ").");
            return;
         }
      }

      MessageTransformer.transform(objectToWrite, objectTransform);
      recordObject(timestamp, objectToWrite);
   }

   public void recordObject(long timestamp, Object object) throws IOException
   {
      outputStream.writeLong(timestamp);
      outputStream.writeObject(object);
      outputStream.flush();
   }

   public void close()
   {
      try
      {
         outputStream.close();
         fileWriter.close();
      }
      catch (IOException e)
      {
         System.err.println(e);
      }
   }

   public void recordList(ArrayList<ScriptObject> scriptObjects) throws IOException
   {
      for (ScriptObject scriptObject : scriptObjects)
      {
         recordObject(scriptObject.getTimeStamp(), scriptObject.getScriptObject());
      }
   }
}

package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.ArrayList;

import com.thoughtworks.xstream.XStream;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.TopicDataType;

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
    * @deprecated This method is highly inefficient. The whole scripting needs to be redone to better
    *             implement the transform.
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
            PrintTools.error("The class: " + className + " needs either a copy constructor: " + className + "(" + className + "), or a setter: " + className
                  + "set(" + className + ").");
            return;
         }
      }

      MessageTransformer.transform(objectToWrite, objectTransform);
      recordObject(timestamp, objectToWrite);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public void recordObject(long timestamp, Object object) throws IOException
   {
      if (object instanceof Packet)
         object = createCopyTrimmedToCurrentSize((Packet) object);
      outputStream.writeLong(timestamp);
      outputStream.writeObject(object);
      outputStream.flush();
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static <T extends Packet<T>> T createCopyTrimmedToCurrentSize(T message)
   {
      if (message == null)
         return null;

      Class<? extends Packet> messageClass = message.getClass();
      T copy;
      try
      {
         copy = (T) messageClass.newInstance();
         copy.set(message);

         for (Field field : messageClass.getDeclaredFields())
         {
            Class<?> fieldType = field.getType();

            if (Packet.class.isAssignableFrom(fieldType))
            {
               field.set(copy, createCopyTrimmedToCurrentSize((Packet) field.get(message)));
            }
            else if (IDLSequence.Object.class.isAssignableFrom(fieldType))
            {
               field.set(copy, trimIDLSequenceObject((IDLSequence.Object) field.get(message)));
            }
         }

         return copy;
      }
      catch (InstantiationException | IllegalAccessException | IllegalArgumentException | NoSuchFieldException | SecurityException e)
      {
         e.printStackTrace();
         return message;
      }
   }

   @SuppressWarnings({"rawtypes", "unchecked"})
   private static <T> IDLSequence.Object<T> trimIDLSequenceObject(IDLSequence.Object<T> list)
         throws IllegalArgumentException, IllegalAccessException, NoSuchFieldException, SecurityException
   {
      TopicDataType<T> topicDataType = list.getTopicDataType();
      Class clazz = topicDataType.createData().getClass();
      IDLSequence.Object<T> trimmed = new IDLSequence.Object<>(list.size(), clazz, topicDataType);
      Field allocatorField = trimmed.getClass().getSuperclass().getDeclaredField("allocator");
      allocatorField.setAccessible(true);
      allocatorField.set(trimmed, null); // The allocator is a lambda that can not be recorded
      Field sizeField = trimmed.getClass().getSuperclass().getDeclaredField("size");
      sizeField.setAccessible(true);
      sizeField.set(trimmed, list.size());
      Field valuesField = trimmed.getClass().getSuperclass().getDeclaredField("values");
      valuesField.setAccessible(true);
      Object[] array = list.toArray();
      if (Packet.class.isAssignableFrom(clazz))
      {
         for (int i = 0; i < array.length; i++)
         {
            array[i] = createCopyTrimmedToCurrentSize((Packet) array[i]);
         }
      }
      valuesField.set(trimmed, array);

      return trimmed;
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

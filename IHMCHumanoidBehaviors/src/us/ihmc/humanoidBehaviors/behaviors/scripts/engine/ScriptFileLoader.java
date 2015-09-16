package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.util.ArrayList;

import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import com.thoughtworks.xstream.XStream;
import com.thoughtworks.xstream.io.StreamException;
import com.thoughtworks.xstream.mapper.MapperWrapper;

public class ScriptFileLoader
{

   private final FileReader fileReader;
   private ObjectInputStream inputStream;

   public ScriptFileLoader(String filename) throws IOException, StreamException
   {
      File file = new File(filename);
      if (!file.exists())
      {
         throw new IOException("Unknown file " + filename);
      }
      XStream xStream = new XStream()
      {
         protected MapperWrapper wrapMapper(MapperWrapper next)
         {
            return new MapperWrapper(next)
            {

               public boolean shouldSerializeMember(@SuppressWarnings("rawtypes") Class definedIn, String fieldName)
               {
                  return definedIn != Object.class ? super.shouldSerializeMember(definedIn, fieldName) : false;
               }
            };
         }
      };

      fileReader = new FileReader(file.getAbsoluteFile());
      
      inputStream = xStream.createObjectInputStream(fileReader);
   }
   
   public ScriptFileLoader(InputStream scriptInputStream) throws IOException
   {
	   XStream xStream = new XStream()
	      {
	         protected MapperWrapper wrapMapper(MapperWrapper next)
	         {
	            return new MapperWrapper(next)
	            {

	               public boolean shouldSerializeMember(@SuppressWarnings("rawtypes") Class definedIn, String fieldName)
	               {
	                  return definedIn != Object.class ? super.shouldSerializeMember(definedIn, fieldName) : false;
	               }
	            };
	         }
	      };
	      
	      fileReader = null;
	      inputStream = xStream.createObjectInputStream(scriptInputStream);
   }

   public void close()
   {
      try
      {
    	  if(fileReader != null)
    	  {
    		  fileReader.close();    		  
    	  }
         inputStream.close();
      }
      catch (IOException e)
      {
         System.err.println(e);
      }
   }

   public Object readObject() throws IOException
   {
      try
      {
         return inputStream.readObject();
      }
      catch (ClassNotFoundException e)
      {
         throw new RuntimeException(e);
      }
   }

   public long getTimestamp() throws IOException
   {
      return inputStream.readLong();
   }

   @SuppressWarnings("rawtypes")
   public Object getObject(RigidBodyTransform transform) throws IOException
   {
      Object object;
      try
      {
         object = inputStream.readObject();
         if (object instanceof TransformableDataObject)
         {
            object = ((TransformableDataObject) object).transform(transform);
         }
         return object;
      }
      catch (ClassNotFoundException e)
      {
         throw new RuntimeException(e);
      }
   }

   public ScriptObject getScriptObject(RigidBodyTransform transform) throws IOException
   {
      long timestamp = getTimestamp();
      Object object = getObject(transform);
      return new ScriptObject(timestamp, object);
   }

   public ArrayList<ScriptObject> readIntoList()
   {
      return readIntoList(new RigidBodyTransform());
   }

   public ArrayList<ScriptObject> readIntoList(RigidBodyTransform transform)
   {
      ArrayList<ScriptObject> list = new ArrayList<ScriptObject>();
      while (true)
      {
         try
         {
            list.add(getScriptObject(transform));
         }
         catch (IOException e)
         {
            break;
         }
      }
      return list;
   }

}

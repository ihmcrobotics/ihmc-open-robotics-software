package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;

import com.thoughtworks.xstream.XStream;
import com.thoughtworks.xstream.io.StreamException;
import com.thoughtworks.xstream.mapper.MapperWrapper;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.tools.io.printing.PrintTools;

public class ScriptFileLoader
{
   private final BufferedReader reader;
   private ObjectInputStream inputStream;
   
   public ScriptFileLoader(Path scriptFilePath) throws IOException, StreamException
   {
      PrintTools.info("Working directory: " + System.getProperty("user.dir"));
      if (!Files.exists(scriptFilePath))
      {
         throw new IOException("!Files.exists(path): " + scriptFilePath.toString());
      }
      XStream xStream = new XStream()
      {
         @Override
         protected MapperWrapper wrapMapper(MapperWrapper next)
         {
            return new MapperWrapper(next)
            {
               @Override
               public boolean shouldSerializeMember(@SuppressWarnings("rawtypes") Class definedIn, String fieldName)
               {
                  return definedIn != Object.class ? super.shouldSerializeMember(definedIn, fieldName) : false;
               }
            };
         }
      };

      reader = Files.newBufferedReader(scriptFilePath);
      inputStream = xStream.createObjectInputStream(reader);
   }
   
   
   public ScriptFileLoader(File file) throws IOException
   {
	   this(new BufferedInputStream(new FileInputStream(file)));
   }
   
   public ScriptFileLoader(InputStream scriptInputStream) throws IOException
   {
	   XStream xStream = new XStream()
	      {
	         @Override
            protected MapperWrapper wrapMapper(MapperWrapper next)
	         {
	            return new MapperWrapper(next)
	            {

	               @Override
                  public boolean shouldSerializeMember(@SuppressWarnings("rawtypes") Class definedIn, String fieldName)
	               {
	                  return definedIn != Object.class ? super.shouldSerializeMember(definedIn, fieldName) : false;
	               }
	            };
	         }
	      };
	      
	      reader = null;
	      inputStream = xStream.createObjectInputStream(scriptInputStream);
   }

   public void close()
   {
      try
      {
    	  if(reader != null)
    	  {
    		  reader.close();    		  
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

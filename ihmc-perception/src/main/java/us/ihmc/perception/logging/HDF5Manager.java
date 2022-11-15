package us.ihmc.perception.logging;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.H5File;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.log.LogTools;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public class HDF5Manager
{

   public static int MAX_BUFFER_SIZE = 500;

   private HashMap<String, Group> groups;
   private HashMap<String, ArrayList<Float>> buffers;
   private HashMap<String, Integer> counts;
   private H5File file;

   public HDF5Manager(String filePath, int flag)
   {
      file = new H5File(filePath, flag);
      groups = new HashMap<>();
      buffers = new HashMap<>();
   }

   public void resetBuffer(String namespace)
   {
      buffers.get(namespace).clear();
//      ArrayList<Float> list = new ArrayList<>();
//      buffers.put(namespace, list);
   }

   public int getBufferIndex(String namespace)
   {
      if (buffers.containsKey(namespace))
      {
         LogTools.info("Get Index: {} {}", namespace, buffers.get(namespace).size());
         return buffers.get(namespace).size();
      }
      else
      {
         LogTools.info("Index Not Found");
         return 0;
      }
   }

   public ArrayList<Float> getBuffer(String namespace)
   {
      if (buffers.containsKey(namespace))
      {
         return buffers.get(namespace);
      }
      else
      {
         ArrayList<Float> list = new ArrayList<>();
         buffers.put(namespace, list);
         return list;
      }
   }

   public Group getGroup(String namespace)
   {
      if (groups.containsKey(namespace))
      {
         return groups.get(namespace);
      }
      else
      {
         LogTools.info("Creating Group: {}", namespace);
         Group group = createGroup(namespace);
         groups.put(namespace, group);
         return group;
      }
   }

   public long getCount(String namespace)
   {
      if (groups.containsKey(namespace))
         return groups.get(namespace).getNumObjs();
      else if (file.nameExists(namespace))
      {
         Group group = file.openGroup(namespace);
         groups.put(namespace, group);
         return group.getNumObjs();
      }
      else
         return 0;
   }

   public Group openGroup(String namespace)
   {
      Group group = file.openGroup(namespace);
      return group;
   }

   public Group createGroup(String namespace)
   {
      Path path = Paths.get(namespace);
      Group group = null;

      for (int i = 0; i < path.getNameCount(); i++)
      {
         String name = path.subpath(0, i + 1).toString();
         if (!file.nameExists(name))
         {
//            LogTools.info("Creating Group: {}", name);
            group = file.createGroup(name);
         }
//         else
//         {
////            LogTools.warn("Not Creating, Exists: /{}", name);
//         }
      }

      if (group == null)
      {
         group = file.openGroup(namespace);
      }

      return group;
   }

   public H5File getFile()
   {
      return file;
   }

   public HashMap<String, Group> getGroups()
   {
      return groups;
   }
}

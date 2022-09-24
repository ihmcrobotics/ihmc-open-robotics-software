package us.ihmc.perception;

import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.H5File;
import us.ihmc.log.LogTools;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;

public class HDF5Manager
{

   private HashMap<String, Group> groups;
   private H5File file;

   public HDF5Manager(String filePath, int flag)
   {
      file = new H5File(filePath, flag);
      groups = new HashMap<>();
   }

   public Group getGroup(String namespace)
   {
      if (groups.containsKey(namespace))
      {
         return groups.get(namespace);
      }
      else
      {
         Group group = createGroup(namespace);
         groups.put(namespace, group);
         return group;
      }
   }

   public long getCount(String namespace)
   {
      if (groups.containsKey(namespace))
         return groups.get(namespace).getNumObjs();
      else
         return 0;
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
         else
         {
//            LogTools.warn("Not Creating, Exists: /{}", name);
         }
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

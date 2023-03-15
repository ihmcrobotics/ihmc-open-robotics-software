package us.ihmc.perception.logging;

import org.bytedeco.hdf5.*;
import us.ihmc.log.LogTools;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

/**
 * Handler for an HDF5 file. Also manages intermediate buffers, index counts, and group handles.
 *
 * @author Bhavyansh Mishra
 */
public class HDF5Manager
{
   private HashMap<String, Group> groups;
   private H5File file;

   /**
    * Constructor for HDF5Manager
    *
    * @param filePath Absolute path to the HDF5 file to be managed
    * @param flag     The read-write flag to select the mode in which the file needs to be managed
    */
   public HDF5Manager(String filePath, int flag)
   {
      file = new H5File(filePath, flag);
      groups = new HashMap<>();
   }


   /**
    * Open a handle for the group requested in the namespace
    *
    * @param namespace The namespace for which a group handle has been requested
    * @return The HDF5 group handle for the requested namespace
    */
   public Group openOrGetGroup(String namespace)
   {
      if (groups.containsKey(namespace))
      {
         return groups.get(namespace);
      }
      else if (file.nameExists(namespace))
      {
         LogTools.info("Opening Group: {}", namespace);
         Group group = file.openGroup(namespace);
         groups.put(namespace, group);
         return group;
      }
      else
         return null;
   }

   /**
    * Getter to request the HDF5 group for the given topic name
    *
    * @param namespace The namespace or topic name for which the HDF5 group object has been requested
    * @return The HDF5 group object associated with the given namespace or topic name.
    */
   public Group createOrGetGroup(String namespace)
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

   /**
    * Get file-count for the provided namespace
    *
    * @param namespace The namespace or topic name for which the file count has been requested
    * @return The file count for the namespace or topic name requested
    */
   public int getCount(String namespace)
   {
      int count = 0;
      if (groups.containsKey(namespace))
      {
         count = (int) groups.get(namespace).getNumObjs();
         return count;
      }
      else if (file.nameExists(namespace))
      {
         Group group = file.openGroup(namespace);
         groups.put(namespace, group);
         count = (int) group.getNumObjs();
         return count;
      }
      else
         return 0;
   }

   /**
    * Create a new handle for the group requested in the namespace
    *
    * @param namespace The namespace for which a group handle has been requested
    * @return The HDF5 group handle for the requested namespace
    */
   public Group createGroup(String namespace)
   {
      Path path = Paths.get(namespace);

      Group group = null;

      if(file.nameExists("file"))
         group = file.openGroup("file");
      else
      {
         group = file.createGroup("file");
      }

      for (int i = 0; i < path.getNameCount(); i++)
      {
         String name = "/" + path.subpath(0, i + 1).toString();

         LogTools.info("Creating Group: {}", name);

         if(group.nameExists(name))
            continue;

         group = group.createGroup(name);
      }

      if (group.getObjName().getString().equals("root"))
      {
         group = file.openGroup(namespace);
      }

      return group;
   }

   public void closeFile()
   {
      if (file != null)
      {
         for (Group group : groups.values())
         {
            group._close();
         }
         file._close();
         file = null;
      }
   }

   public H5File getFile()
   {
      return file;
   }

   public Map<String, Group> getGroups()
   {
      return groups;
   }
}

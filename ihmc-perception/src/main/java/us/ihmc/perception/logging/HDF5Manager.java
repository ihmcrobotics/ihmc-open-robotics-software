package us.ihmc.perception.logging;

import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.H5File;
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

   public static int MAX_BUFFER_SIZE = 10;

   private HashMap<String, Group> groups;
   private HashMap<String, TFloatArrayList> floatBuffers;
   private HashMap<String, TLongArrayList> longBuffers;
   private HashMap<String, Integer> counts;
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
      floatBuffers = new HashMap<>();
      longBuffers = new HashMap<>();
   }

   /**
    * Resets buffers for storing intermediate data before logging.
    *
    * @param namespace The string namespace or topic name for which the buffer needs to be reset
    */
   public void resetBuffer(String namespace)
   {
      floatBuffers.get(namespace).clear();
   }

   /**
    * Get the current index within the provided namespace. The number of files in the directory decides the current index value
    *
    * @param namespace The string namespace or topic name for which the file-count index needs to be returned
    * @return File count index for the given namespace or topic name
    */
   public int getBufferIndex(String namespace)
   {
      if (floatBuffers.containsKey(namespace))
      {
         LogTools.info("Get Index: {} {}", namespace, floatBuffers.get(namespace).size());
         return floatBuffers.get(namespace).size();
      }
      else
      {
         LogTools.info("Index Not Found");
         return 0;
      }
   }

   /**
    * Getter for Buffer belonging to the requested namespace
    *
    * @param namespace The string namespace or topic name for which the buffer has been requested
    * @return Intermediate buffer for the requested namespace or topic name
    */
   public TFloatArrayList getFloatBuffer(String namespace)
   {
      if (floatBuffers.containsKey(namespace))
      {
         return floatBuffers.get(namespace);
      }
      else
      {
         TFloatArrayList list = new TFloatArrayList();
         floatBuffers.put(namespace, list);
         return list;
      }
   }

   /**
    * Getter for Long Buffers belonging to the requested namespace
    *
    * @param namespace The string namespace or topic name for which the buffer has been requested
    * @return Long intermediate buffer for the requested namespace or topic name
    */
   public TLongArrayList getLongBuffer(String namespace)
   {
      if (longBuffers.containsKey(namespace))
      {
         return longBuffers.get(namespace);
      }
      else
      {
         TLongArrayList list = new TLongArrayList();
         longBuffers.put(namespace, list);
         return list;
      }
   }

   /**
    * Getter to request the HDF5 group for the given topic name
    *
    * @param namespace The namespace or topic name for which the HDF5 group object has been requested
    * @return The HDF5 group object associated with the given namespace or topic name.
    */
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

   /**
    * Get file-count for the provided namespace
    *
    * @param namespace The namespace or topic name for which the file count has been requested
    * @return The file count for the namespace or topic name requested
    */
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

   /**
    * Open a handle for the group requested in the namespace
    *
    * @param namespace The namespace for which a group handle has been requested
    * @return The HDF5 group handle for the requested namespace
    */
   public Group openGroup(String namespace)
   {
      Group group = file.openGroup(namespace);
      return group;
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

      for (int i = 0; i < path.getNameCount(); i++)
      {
         String name = path.subpath(0, i + 1).toString();
         if (!file.nameExists(name))
         {
            group = file.createGroup(name);
         }
      }

      if (group == null)
      {
         group = file.openGroup(namespace);
      }

      return group;
   }

   public void closeFile()
   {
      for(Group group : groups.values())
      {
         group.close();
      }
      file.close();
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

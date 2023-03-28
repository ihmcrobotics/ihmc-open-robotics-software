package us.ihmc.perception.objects;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.*;

public class ObjectInfo
{
   protected final ArrayList<Integer> IDs = new ArrayList<>();
   protected final ArrayList<String> objectNames = new ArrayList<>();
   protected final HashMap<String, String> modelFileName = new HashMap<>();
   protected WorkspaceResourceFile configurationFile;

   public ObjectInfo()
   {
      // read parameters regarding the properties of available objects with ArUco markers attached
      String file = "ObjectsInfo.json";
      WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(getClass(), "/us/ihmc/perception/objects");
      configurationFile = new WorkspaceResourceFile(directory, file);
   }

   public void load()
   {
      LogTools.info("Loading parameters from resource: {}", configurationFile.getFileName());
      JSONFileTools.load(configurationFile, jsonNode ->
      {
         JsonNode objectsArrayNode = jsonNode.get("objects");
         int size = objectsArrayNode.size();
         //iterating objects
         for (int i = 0; i < size; i++)
         {
            JsonNode objectNode = objectsArrayNode.get(i);
            objectNames.add(objectNode.get("name").asText());
            JsonNode propertiesArrayNode = objectNode.get("properties");
            for (JsonNode propertyObject : propertiesArrayNode)
            {
               IDs.add(propertyObject.get("ID").asInt());
               modelFileName.put(objectNames.get(i), propertyObject.get("modelFileName").asText());
            }
         }
      });
   }

   public String getObjectName(int id)
   {
      return objectNames.get(IDs.indexOf(id));
   }

   public int getNumberOfObjects()
   {
      return objectNames.size();
   }

   public ArrayList<String> getObjectNames()
   {
      return objectNames;
   }

   public ArrayList<Integer> getIDs()
   {
      return IDs;
   }

   public String getModelFileName(String objectName)
   {
      return modelFileName.get(objectName);
   }
}

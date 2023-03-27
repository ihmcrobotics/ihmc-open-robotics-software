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
   protected final List<String> objectNames = new ArrayList<>();
   protected double[][] bodyToAppendixTranslations;
   protected double[][] bodyToAppendixRotations;
   protected final HashMap<String, String> virtualBodyFileName = new HashMap<>();
   // appendix is for multi-body systems with dofs (TODO make appendix an array in case the body has more than 1 dof)
   protected final HashMap<String, String> virtualAppendixFileName = new HashMap<>();
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
         bodyToAppendixTranslations = new double[size][3];
         bodyToAppendixRotations = new double[size][3];
         //iterating objects
         for (int i = 0; i < size; i++)
         {
            JsonNode objectNode = objectsArrayNode.get(i);
            objectNames.add(objectNode.get("name").asText());
            JsonNode propertiesArrayNode = objectNode.get("properties");
            for (JsonNode propertyObject : propertiesArrayNode)
            {
               JsonNode tempArrayNode = propertyObject.get("translationMainBodyToAppendix");
               for (int j = 0; j < 3; j++)
                  bodyToAppendixTranslations[i][j] = tempArrayNode.get(j).asDouble();
               tempArrayNode = propertyObject.get("yawPitchRollMainBodyToAppendix");
               for (int j = 0; j < 3; j++)
                  bodyToAppendixRotations[i][j] = tempArrayNode.get(j).asDouble();
               virtualBodyFileName.put(objectNames.get(i), propertyObject.get("virtualMainBodyFileName").asText());
               virtualAppendixFileName.put(objectNames.get(i), propertyObject.get("virtualAppendixFileName").asText());
            }
         }
      });
   }

   public Point3D getAppendixTranslation(String objectName)
   {
      int index = objectNames.indexOf(objectName);
      return new Point3D(bodyToAppendixTranslations[index][0], bodyToAppendixTranslations[index][1], bodyToAppendixTranslations[index][2]);
   }

   public YawPitchRoll getAppendixYawPitchRoll(String objectName)
   {
      int index = objectNames.indexOf(objectName);
      return new YawPitchRoll(bodyToAppendixRotations[index][0], bodyToAppendixRotations[index][1], bodyToAppendixRotations[index][2]);
   }

   public String getVirtualBodyFileName(String objectName)
   {
      return virtualBodyFileName.get(objectName);
   }

   public String getVirtualAppendixFileName(String objectName)
   {
      return virtualAppendixFileName.get(objectName);
   }

   public boolean hasAppendix(String objectName)
   {
      return virtualAppendixFileName.containsKey(objectName);
   }
}

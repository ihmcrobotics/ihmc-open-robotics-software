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
   private final List<Integer> markerIds = new ArrayList<>();
   private String[] objectNames;
   private double[] markerSizes;
   private double[][] markerToBodyTranslations;
   private double[][] markerToBodyRotations;
   private double[][] bodyToAppendixTranslations;
   private double[][] bodyToAppendixRotations;
   private final HashMap<String, String> virtualBodyFileName = new HashMap<>();
   private final HashMap<String, String> virtualAppendixFileName = new HashMap<>();

   public ObjectInfo()
   {
      // read parameters regarding the properties of available objects with ArUco markers attached
      String configurationFile = "ObjectsInfo.json";
      LogTools.info("Loading parameters from resource: {}", configurationFile);
      WorkspaceResourceDirectory directory = new WorkspaceResourceDirectory(getClass(), "/us/ihmc/perception/objects");
      WorkspaceResourceFile file = new WorkspaceResourceFile(directory, configurationFile);
      JSONFileTools.load(file, jsonNode ->
      {
         JsonNode objectsArrayNode = jsonNode.get("objects");
         int size = objectsArrayNode.size();
         objectNames = new String[size];
         markerSizes = new double[size];
         markerToBodyTranslations = new double[size][3];
         markerToBodyRotations = new double[size][3];
         bodyToAppendixTranslations = new double[size][3];
         bodyToAppendixRotations = new double[size][3];
         //iterating objects
         for (int i=0; i<size; i++)
         {
            JsonNode objectNode = objectsArrayNode.get(i);
            Iterator<Map.Entry<String, JsonNode>> objectFields = objectNode.fields();
            while (objectFields.hasNext()) {
               Map.Entry<String, JsonNode> objectPropertyMap = objectFields.next();
               switch (objectPropertyMap.getKey()) {
                  case "name" -> objectNames[i] = objectPropertyMap.getValue().asText();
                  case "properties" -> {
                     JsonNode propertiesArrayNode = objectPropertyMap.getValue();
                     for (JsonNode propertyObject : propertiesArrayNode) {
                        Iterator<Map.Entry<String, JsonNode>> propertyFields = propertyObject.fields();
                        while (propertyFields.hasNext()) {
                           Map.Entry<String, JsonNode> propertiesMap = propertyFields.next();
                           switch (propertiesMap.getKey()) {
                              case "markerId" -> markerIds.add(propertiesMap.getValue().asInt());
                              case "markerSize" -> markerSizes[i] = propertiesMap.getValue().asDouble();
                              case "translationMarkerToMainBody" -> {
                                 JsonNode translationArrayNode = propertiesMap.getValue();
                                 for (int j = 0; j < 3; j++) {
                                    markerToBodyTranslations[i][j] = translationArrayNode.get(j).asDouble();
                                 }
                              }
                              case "yawPitchRollMarkerToMainBody" ->
                              {
                                 JsonNode rotationArrayNode = propertiesMap.getValue();
                                 for (int j=0; j<3; j++)
                                    markerToBodyRotations[i][j] = rotationArrayNode.get(j).asDouble();
                              }
                              case "translationMainBodyToAppendix" ->
                              {
                                 JsonNode translationArrayNode = propertiesMap.getValue();
                                 for (int j=0; j<3; j++)
                                    bodyToAppendixTranslations[i][j] = (double) translationArrayNode.get(j).asDouble();
                              }
                              case "yawPitchRollMainBodyToAppendix" ->
                              {
                                 JsonNode rotationArrayNode = propertiesMap.getValue();
                                 for (int j=0; j<3; j++)
                                    bodyToAppendixRotations[i][j] = (double) rotationArrayNode.get(j).asDouble();
                              }
                              case "virtualMainBodyFileName" ->
                              {
                                 // insert in the hashmap the last objectName and the filename
                                 virtualBodyFileName.put(objectNames[i], propertiesMap.getValue().asText());
                              }
                              case "virtualAppendixFileName" ->
                                    virtualAppendixFileName.put(objectNames[i], propertiesMap.getValue().asText());
                           }
                        }
                     }
                  }
               }
            }
         }
      });
   }

   public List<Integer> getMarkersId()
   {
      return markerIds;
   }

   public double getMarkerSize(int id)
   {
      return markerSizes[markerIds.indexOf(id)];
   }

   public String getObjectName(int id)
   {
      return objectNames[markerIds.indexOf(id)];
   }

   public Point3D getMarkerTranslation(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new Point3D(markerToBodyTranslations[realIndex][0], markerToBodyTranslations[realIndex][1], markerToBodyTranslations[realIndex][2]);
   }

   public YawPitchRoll getMarkerYawPitchRoll(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new YawPitchRoll(markerToBodyRotations[realIndex][0], markerToBodyRotations[realIndex][1], markerToBodyRotations[realIndex][2]);
   }

   public Point3D getAppendixTranslation(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new Point3D(bodyToAppendixTranslations[realIndex][0], bodyToAppendixTranslations[realIndex][1], bodyToAppendixTranslations[realIndex][2]);
   }

   public YawPitchRoll getAppendixYawPitchRoll(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new YawPitchRoll(bodyToAppendixRotations[realIndex][0], bodyToAppendixRotations[realIndex][1], bodyToAppendixRotations[realIndex][2]);
   }

   public String getVirtualBodyFileName(String objectName)
   {
      return virtualBodyFileName.get(objectName);
   }

   public String getVirtualAppendixFileName(String objectName)
   {
      return virtualAppendixFileName.get(objectName);
   }

   public boolean hasAppendix(int id)
   {
      return virtualAppendixFileName.containsKey(getObjectName(id));
   }

   public boolean hasAppendix(String objectName)
   {
      return virtualAppendixFileName.containsKey(objectName);
   }
}

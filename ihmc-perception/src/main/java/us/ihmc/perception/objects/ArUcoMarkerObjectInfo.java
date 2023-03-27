package us.ihmc.perception.objects;

import com.fasterxml.jackson.databind.JsonNode;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.util.ArrayList;
import java.util.List;

public class ArUcoMarkerObjectInfo extends ObjectInfo
{
   private double[] markerSizes;
   private double[][] markerToBodyTranslations;
   private double[][] markerToBodyRotations;

   public ArUcoMarkerObjectInfo()
   {
      super();
   }

   public void load()
   {
      LogTools.info("Loading parameters from resource: {}", configurationFile.getFileName());
      WorkspaceResourceFile configurationFile = super.configurationFile;
      JSONFileTools.load(configurationFile, jsonNode ->
      {
         JsonNode objectsArrayNode = jsonNode.get("objects");
         int size = objectsArrayNode.size();
         markerSizes = new double[size];
         markerToBodyTranslations = new double[size][3];
         markerToBodyRotations = new double[size][3];
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
               IDs.add(propertyObject.get("ID").asInt());
               markerSizes[i] = propertyObject.get("markerSize").asDouble();
               JsonNode tempArrayNode = propertyObject.get("translationMarkerToMainBody");
               for (int j = 0; j < 3; j++)
               {
                  markerToBodyTranslations[i][j] = tempArrayNode.get(j).asDouble();
               }
               tempArrayNode = propertyObject.get("yawPitchRollMarkerToMainBody");
               for (int j = 0; j < 3; j++)
                  markerToBodyRotations[i][j] = tempArrayNode.get(j).asDouble();
               tempArrayNode = propertyObject.get("translationMainBodyToAppendix");
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

   public List<Integer> getMarkersId()
   {
      return IDs;
   }

   public double getMarkerSize(int id)
   {
      return markerSizes[IDs.indexOf(id)];
   }

   public Point3D getMarkerTranslation(int id)
   {
      int realIndex = IDs.indexOf(id);
      return new Point3D(markerToBodyTranslations[realIndex][0], markerToBodyTranslations[realIndex][1], markerToBodyTranslations[realIndex][2]);
   }

   public YawPitchRoll getMarkerYawPitchRoll(int id)
   {
      int realIndex = IDs.indexOf(id);
      return new YawPitchRoll(markerToBodyRotations[realIndex][0], markerToBodyRotations[realIndex][1], markerToBodyRotations[realIndex][2]);
   }

   public Point3D getAppendixTranslation(int id)
   {
      int realIndex = IDs.indexOf(id);
      return new Point3D(bodyToAppendixTranslations[realIndex][0], bodyToAppendixTranslations[realIndex][1], bodyToAppendixTranslations[realIndex][2]);
   }

   public YawPitchRoll getAppendixYawPitchRoll(int id)
   {
      int realIndex = IDs.indexOf(id);
      return new YawPitchRoll(bodyToAppendixRotations[realIndex][0], bodyToAppendixRotations[realIndex][1], bodyToAppendixRotations[realIndex][2]);
   }

   public boolean hasAppendix(int id)
   {
      return virtualAppendixFileName.containsKey(getObjectName(id));
   }
}

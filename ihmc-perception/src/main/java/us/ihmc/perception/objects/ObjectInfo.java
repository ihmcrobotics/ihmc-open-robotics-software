package us.ihmc.perception.objects;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
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
      try
      {
         String configurationFile = "ObjectsInfo.json";
         LogTools.info("Loading parameters from resource: {}", configurationFile);

         JSONObject jsonObject = (JSONObject) new JSONParser().parse(new InputStreamReader(getClass().getResourceAsStream(configurationFile)));
         // getting objects
         JSONArray objectsArray = (JSONArray) jsonObject.get("objects");
         int size = objectsArray.size();
         objectNames = new String[size];
         markerSizes = new double[size];
         markerToBodyTranslations = new double[size][3];
         markerToBodyRotations = new double[size][3];
         bodyToAppendixTranslations = new double[size][3];
         bodyToAppendixRotations = new double[size][3];
         //iterating objects
         for (int i=0; i<size; i++)
         {
            for (Map.Entry objectPropertyMap : (Iterable<Map.Entry>) ((Map) objectsArray.get(i)).entrySet())
            {
               switch (objectPropertyMap.getKey().toString())
               {
                  case "name" -> objectNames[i] = (String) objectPropertyMap.getValue();
                  case "properties" ->
                  {
                     JSONArray propertiesArray = (JSONArray) objectPropertyMap.getValue();
                     // iterate over properties
                     for (Object propertyObject : propertiesArray)
                     {
                        for (Map.Entry propertiesMap : (Iterable<Map.Entry>) ((Map) propertyObject).entrySet())
                        {
                           switch (propertiesMap.getKey().toString())
                           {
                              case "markerId" -> markerIds.add((int) ((long) propertiesMap.getValue()));
                              case "markerSize" -> markerSizes[i] = (double) propertiesMap.getValue();
                              case "translationMarkerToMainBody" ->
                              {
                                 JSONArray translationArray = (JSONArray) propertiesMap.getValue();
                                 for (int j=0; j<3; j++)
                                    markerToBodyTranslations[i][j] = (double) translationArray.get(j);
                              }
                              case "yawPitchRollMarkerToMainBody" ->
                              {
                                 JSONArray rotationArray = (JSONArray) propertiesMap.getValue();
                                 for (int j=0; j<3; j++)
                                    markerToBodyRotations[i][j] = (double) rotationArray.get(j);
                              }
                              case "translationMainBodyToAppendix" ->
                              {
                                 JSONArray translationArray = (JSONArray) propertiesMap.getValue();
                                 for (int j=0; j<3; j++)
                                    bodyToAppendixTranslations[i][j] = (double) translationArray.get(j);
                              }
                              case "yawPitchRollMainBodyToAppendix" ->
                              {
                                 JSONArray rotationArray = (JSONArray) propertiesMap.getValue();
                                 for (int j=0; j<3; j++)
                                    bodyToAppendixRotations[i][j] = (double) rotationArray.get(j);
                              }
                              case "virtualMainBodyFileName" ->
                              {
                                 // insert in the hashmap the last objectName and the filename
                                 virtualBodyFileName.put(objectNames[i], (String) propertiesMap.getValue());
                              }
                              case "virtualAppendixFileName" ->
                                    virtualAppendixFileName.put(objectNames[i], (String) propertiesMap.getValue());
                              default ->
                              {
                              }
                           }
                        }
                     }
                  }
                  default ->
                  {
                  }
               }
            }
         }
      }
      catch (FileNotFoundException ex)
      {
         ex.printStackTrace();
      }
      catch (IOException | ParseException e)
      {
         throw new RuntimeException(e);
      }
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

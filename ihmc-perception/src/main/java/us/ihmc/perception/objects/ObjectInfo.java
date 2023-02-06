package us.ihmc.perception.objects;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class ObjectInfo
{
   private final List<Integer> markerIds = new ArrayList<>();
   private final List<String> objectNames = new ArrayList<>();
   private final List<Double> markerSizes = new ArrayList<>();
   private final List<List<Double>> markerToBodyTranslations = new ArrayList<>();
   private final List<List<Double>> markerToBodyRotations = new ArrayList<>();
   private final List<List<Double>> bodyToAppendixTranslations = new ArrayList<>();
   private final List<List<Double>> bodyToAppendixRotations = new ArrayList<>();
   private final HashMap<String, String> virtualBodyFileName = new HashMap<>();
   private final HashMap<String, String> virtualAppendixFileName = new HashMap<>();

   public ObjectInfo()
   {
      // read parameters regarding the properties of available objects with ArUco markers attached
      try
      {
         WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/main/resources");
         String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
         String configurationFile = "/us/ihmc/perception/objects/ObjectsInfo.json";
         String demoDirectory = directoryAbsolutePath + configurationFile;
         Path pathFile = Paths.get(demoDirectory);
         LogTools.info("Loading parameters from resource: {}", configurationFile);

         JSONObject jsonObject = (JSONObject) new JSONParser().parse(new FileReader(pathFile.toAbsolutePath().toString()));
         // getting objects
         JSONArray objectsArray = (JSONArray) jsonObject.get("objects");
         //iterating objects
         for (Object objectElement : objectsArray)
         {
            for (Map.Entry objectPropertyMap : (Iterable<Map.Entry>) ((Map) objectElement).entrySet())
            {
               switch (objectPropertyMap.getKey().toString())
               {
                  case "name" -> objectNames.add((String) objectPropertyMap.getValue());
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
                              case "markerSize" -> markerSizes.add((double) propertiesMap.getValue());
                              case "translationMarkerToMainBody" ->
                              {
                                 JSONArray translationArray = (JSONArray) propertiesMap.getValue();
                                 Iterator translationIterator = translationArray.iterator();
                                 List<Double> translation = new ArrayList<>(3);
                                 while (translationIterator.hasNext())
                                    translation.add((Double) translationIterator.next());
                                 markerToBodyTranslations.add(translation);
                              }
                              case "yawPitchRollMarkerToMainBody" ->
                              {
                                 JSONArray rotationArray = (JSONArray) propertiesMap.getValue();
                                 Iterator rotationIterator = rotationArray.iterator();
                                 List<Double> rotation = new ArrayList<>(3);
                                 while (rotationIterator.hasNext())
                                    rotation.add((Double) rotationIterator.next());
                                 markerToBodyRotations.add(rotation);
                              }
                              case "translationMainBodyToAppendix" ->
                              {
                                 JSONArray translationArray = (JSONArray) propertiesMap.getValue();
                                 Iterator translationIterator = translationArray.iterator();
                                 List<Double> translation = new ArrayList<>(3);
                                 while (translationIterator.hasNext())
                                    translation.add((Double) translationIterator.next());
                                 bodyToAppendixTranslations.add(translation);
                              }
                              case "yawPitchRollMainBodyToAppendix" ->
                              {
                                 JSONArray rotationArray = (JSONArray) propertiesMap.getValue();
                                 Iterator rotationIterator = rotationArray.iterator();
                                 List<Double> rotation = new ArrayList<>(3);
                                 while (rotationIterator.hasNext())
                                    rotation.add((Double) rotationIterator.next());
                                 bodyToAppendixRotations.add(rotation);
                              }

                              case "virtualMainBodyFileName" ->
                              {
                                 // insert in the hashmap the last objectName and the filename
                                 virtualBodyFileName.put(objectNames.get(objectNames.size() - 1), (String) propertiesMap.getValue());
                              }
                              case "virtualAppendixFileName" ->
                                    virtualAppendixFileName.put(objectNames.get(objectNames.size() - 1), (String) propertiesMap.getValue());
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
      return markerSizes.get(markerIds.indexOf(id));
   }

   public String getObjectName(int id)
   {
      return objectNames.get(markerIds.indexOf(id));
   }

   public Point3D getMarkerTranslation(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new Point3D(markerToBodyTranslations.get(realIndex).get(0), markerToBodyTranslations.get(realIndex).get(1), markerToBodyTranslations.get(realIndex).get(2));
   }

   public YawPitchRoll getMarkerYawPitchRoll(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new YawPitchRoll(markerToBodyRotations.get(realIndex).get(0), markerToBodyRotations.get(realIndex).get(1), markerToBodyRotations.get(realIndex).get(2));
   }

   public Point3D getAppendixTranslation(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new Point3D(bodyToAppendixTranslations.get(realIndex).get(0), bodyToAppendixTranslations.get(realIndex).get(1), bodyToAppendixTranslations.get(realIndex).get(2));
   }

   public YawPitchRoll getAppendixYawPitchRoll(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new YawPitchRoll(bodyToAppendixRotations.get(realIndex).get(0), bodyToAppendixRotations.get(realIndex).get(1), bodyToAppendixRotations.get(realIndex).get(2));
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

package us.ihmc.perception;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

public class ArUcoObjectInfo
{
   private final List<Integer> markerIds = new ArrayList<>();
   private final List<String> objectNames = new ArrayList<>();
   private final List<Double> markerSizes = new ArrayList<>();
   private final List<List<Double>> objectTranslations = new ArrayList<>();
   private final List<List<Double>> objectRotations = new ArrayList<>();

   public ArUcoObjectInfo()
   {
      // read parameters regarding the properties of available objects with ArUco markers attached
      try
      {
         WorkspaceDirectory directory = new WorkspaceDirectory("ihmc-open-robotics-software", "ihmc-perception/src/main/resources");
         String directoryAbsolutePath = directory.getDirectoryPath().toAbsolutePath().toString();
         String configurationFile = "/us/ihmc/perception/objectDetection/ArUcoObjectsInfo.json";
         String demoDirectory = directoryAbsolutePath + configurationFile;
         Path pathFile = Paths.get(demoDirectory);
         LogTools.info("Loading parameters from resource: {}", configurationFile);

         JSONObject jsonObject = (JSONObject) new JSONParser().parse(new FileReader(pathFile.toAbsolutePath().toString()));
         // getting objects with marker
         JSONArray objectsArray = (JSONArray) jsonObject.get("objectsWithMarker");
         //iterating objects with marker
         for (Object objectElement : objectsArray)
         {
            for (Map.Entry objectPropertyMap : (Iterable<Map.Entry>) ((Map) objectElement).entrySet())
            {
               switch (objectPropertyMap.getKey().toString())
               {
                  case "markerId" -> markerIds.add((int) ((long) objectPropertyMap.getValue()));
                  case "markerSize" -> markerSizes.add((double) objectPropertyMap.getValue());
                  case "objectName" -> objectNames.add((String) objectPropertyMap.getValue());
                  case "translationToMarker" ->
                  {
                     JSONArray translationArray = (JSONArray) objectPropertyMap.getValue();
                     Iterator translationIterator = translationArray.iterator();
                     List<Double> translation = new ArrayList<>(3);
                     while (translationIterator.hasNext())
                        translation.add((Double) translationIterator.next());
                     objectTranslations.add(translation);
                  }
                  case "yawPitchRollToMarker" ->
                  {
                     JSONArray rotationArray = (JSONArray) objectPropertyMap.getValue();
                     Iterator rotationIterator = rotationArray.iterator();
                     List<Double> rotation = new ArrayList<>(3);
                     while (rotationIterator.hasNext())
                        rotation.add((Double) rotationIterator.next());
                     objectRotations.add(rotation);
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

   public Point3D getObjectTranslation(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new Point3D(objectTranslations.get(realIndex).get(0), objectTranslations.get(realIndex).get(1), objectTranslations.get(realIndex).get(2));
   }

   public YawPitchRoll getObjectYawPitchRoll(int id)
   {
      int realIndex = markerIds.indexOf(id);
      return new YawPitchRoll(objectRotations.get(realIndex).get(0), objectRotations.get(realIndex).get(1), objectRotations.get(realIndex).get(2));
   }
}

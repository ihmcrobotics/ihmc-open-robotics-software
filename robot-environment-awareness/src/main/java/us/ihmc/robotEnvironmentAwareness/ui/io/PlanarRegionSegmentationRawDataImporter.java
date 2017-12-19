package us.ihmc.robotEnvironmentAwareness.ui.io;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;

public class PlanarRegionSegmentationRawDataImporter
{
   private final File dataFolder;
   private List<PlanarRegionSegmentationRawData> planarRegionSegmentationRawData = new ArrayList<>();

   public static PlanarRegionSegmentationRawDataImporter createImporterWithFileChooser(Window ownerWindow)
   {
      DirectoryChooser directoryChooser = new DirectoryChooser();

      File initialDirectory = new File("../../Data/Segmentation");
      if (!initialDirectory.exists() || !initialDirectory.isDirectory())
         initialDirectory = new File(".");
      directoryChooser.setInitialDirectory(initialDirectory);

      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return null;
      else
         return new PlanarRegionSegmentationRawDataImporter(result);
   }

   public PlanarRegionSegmentationRawDataImporter(File dataFolder)
   {
      this.dataFolder = dataFolder;
   }

   public void loadPlanarRegionSegmentationData() throws IOException
   {
      File headerFile = new File(dataFolder, "header.txt");
      loadHeader(headerFile);
      loadAllRegions();
   }

   private void loadHeader(File headerFile) throws IOException
   {
      FileReader fileReader = new FileReader(headerFile);
      BufferedReader bufferedReader = new BufferedReader(fileReader);
      String line = "";
      String cvsSplitBy = ",";

      while ((line = bufferedReader.readLine()) != null)
      {
         line = line.replaceAll("regionId: ", "");
         line = line.replaceAll("origin: ", "");
         line = line.replaceAll("normal: ", "");
         String[] values = line.split(cvsSplitBy);
         
         int regionId = Integer.parseInt(values[0]);
         float xOrigin = Float.parseFloat(values[1]);
         float yOrigin = Float.parseFloat(values[2]);
         float zOrigin = Float.parseFloat(values[3]);
         float xNormal = Float.parseFloat(values[4]);
         float yNormal = Float.parseFloat(values[5]);
         float zNormal = Float.parseFloat(values[6]);

         Point3D32 origin = new Point3D32(xOrigin, yOrigin, zOrigin);
         Vector3D32 normal = new Vector3D32(xNormal, yNormal, zNormal);
         PlanarRegionSegmentationRawData rawData = new PlanarRegionSegmentationRawData(regionId, normal, origin);
         planarRegionSegmentationRawData.add(rawData);
      }

      bufferedReader.close();
   }

   private void loadAllRegions()
   {
      planarRegionSegmentationRawData = planarRegionSegmentationRawData.parallelStream().map(this::loadRegion).collect(Collectors.toList());
   }

   private PlanarRegionSegmentationRawData loadRegion(PlanarRegionSegmentationRawData regionToLoad)
   {
      try
      {
         File regionFile = new File(dataFolder, "region" + Integer.toString(regionToLoad.getRegionId()));
         FileReader fileReader = new FileReader(regionFile);
         BufferedReader bufferedReader = new BufferedReader(fileReader);

         String line = "";
         String cvsSplitBy = ",";

         List<Point3D> loadedPoints = new ArrayList<>();
         
         while ((line = bufferedReader.readLine()) != null)
         {
            String[] coordsAsString = line.split(cvsSplitBy);
            float x = Float.parseFloat(coordsAsString[0]);
            float y = Float.parseFloat(coordsAsString[1]);
            float z = Float.parseFloat(coordsAsString[2]);
            loadedPoints.add(new Point3D(x, y, z));
         }

         bufferedReader.close();

         int regionId = regionToLoad.getRegionId();
         Vector3D normal = regionToLoad.getNormal();
         Point3D origin = regionToLoad.getOrigin();
         return new PlanarRegionSegmentationRawData(regionId, normal, origin, loadedPoints);
      }
      catch (IOException e)
      {
         return null;
      }
   }

   public List<PlanarRegionSegmentationRawData> getPlanarRegionSegmentationRawData()
   {
      return planarRegionSegmentationRawData;
   }
}

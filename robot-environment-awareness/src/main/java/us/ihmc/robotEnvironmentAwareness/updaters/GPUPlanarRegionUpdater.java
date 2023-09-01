package us.ihmc.robotEnvironmentAwareness.updaters;

import geometry_msgs.Point;
import map_sense.RawGPUPlanarRegion;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ScanPointFilter;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.converters.ScanRegionFilter;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationDataExporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;

import java.io.File;
import java.util.List;
import java.util.stream.Collectors;

public class GPUPlanarRegionUpdater
{
   private static final boolean EXPORT_SEGMENTATION_ON_EXCEPTION = true;

   private ConcaveHullFactoryParameters concaveHullFactoryParameters;
   private PolygonizerParameters polygonizerParameters;

   public ConcaveHullFactoryParameters getConcaveHullFactoryParameters()
   {
      return concaveHullFactoryParameters;
   }

   public PolygonizerParameters getPolygonizerParameters()
   {
      return polygonizerParameters;
   }

   private final PlanarRegionSegmentationDataExporter dataExporter = EXPORT_SEGMENTATION_ON_EXCEPTION
         ? new PlanarRegionSegmentationDataExporter(new File("DataThrowingException/Segmentation"))
         : null;

   public GPUPlanarRegionUpdater()
   {
      concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
      concaveHullFactoryParameters.setEdgeLengthThreshold(0.224);
      concaveHullFactoryParameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      concaveHullFactoryParameters.setAllowSplittingConcaveHull(false);
      concaveHullFactoryParameters.setMaxNumberOfIterations(5000);
      concaveHullFactoryParameters.setTriangulationTolerance(0.0);

      polygonizerParameters = new PolygonizerParameters();
   }

   public void attachROS2Tuner(ROS2NodeInterface ros2Node)
   {
      new IHMCROS2Callback<>(ros2Node,
                             PerceptionAPI.CONCAVE_HULL_FACTORY_PARAMETERS,
                             parameters -> concaveHullFactoryParameters.setFromColonCommaString(parameters.getParameters().toString()));
      new IHMCROS2Callback<>(ros2Node,
                             PerceptionAPI.POLYGONIZER_PARAMETERS,
                             parameters -> polygonizerParameters.setFromColonCommaString(parameters.getParameters().toString()));
   }

   public void logRawGPURegions(RawGPUPlanarRegionList rawGPUPlanarRegionList)
   {
      for (int i = 0; i < rawGPUPlanarRegionList.getNumOfRegions(); i++)
      {
         RawGPUPlanarRegion rawRegion = rawGPUPlanarRegionList.getRegions().get(i);
         LogTools.info("\tRegion: {}, Size:{}", rawRegion.getId(), rawRegion.getVertices().size());
         LogTools.info("\tNormal: ({},{},{})", rawRegion.getNormal().getX(), rawRegion.getNormal().getY(), rawRegion.getNormal().getZ());
         LogTools.info("\tCentroid: ({},{},{})", rawRegion.getNormal().getX(), rawRegion.getNormal().getY(), rawRegion.getNormal().getZ());
         for (int j = 0; j < rawRegion.getVertices().size(); j++)
         {
            Point point = rawRegion.getVertices().get(j);
            System.out.println(point.getX() + "," + point.getY());
         }
      }
   }

   public void logSegmentationRawData(List<PlanarRegionSegmentationRawData> rawData)
   {
      LogTools.info("Raw Data Regions: {}", rawData.size());
      for (int i = 0; i < rawData.size(); i++)
      {
         PlanarRegionSegmentationRawData rawRegion = rawData.get(i);
         LogTools.info("\tRegion: {}, Size:{}", rawRegion.getRegionId(), rawRegion.getPointCloudInWorld().size());
         //            LogTools.info("\tNormal: ({},{},{})", rawRegion.getNormal().getX(), rawRegion.getNormal().getY(), rawRegion.getNormal().getZ());
         LogTools.info("\tOrigin: ({},{},{})", rawRegion.getOrigin().getX(), rawRegion.getOrigin().getY(), rawRegion.getOrigin().getZ());
         //            for (int j = 0; j < rawRegion.getPointCloudInWorld().size(); j++) {
         //                Point3D point = rawRegion.getPointCloudInWorld().get(j);
         //                System.out.println(point.getX() + "," + point.getY());
         //            }
      }
   }

   public void logFinalGPUPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      LogTools.info("Final GPU Regions: {}", planarRegionsList.getNumberOfPlanarRegions());
      for (PlanarRegion region : planarRegionsList.getPlanarRegionsAsList())
      {
         LogTools.info("\tFinal Region: ID:{} CONVEX:{} HULL:{}", region.getRegionId(), region.getNumberOfConvexPolygons(), region.getConcaveHullSize());
         //            LogTools.info("\tRegion Normal: ({},{},{})", region.getNormal().getX(), region.getNormal().getY(), region.getNormal().getZ());
         Point3D origin = new Point3D();
         region.getOrigin(origin);
         LogTools.info("\tRegion Origin: ({},{},{})", origin.getX(), origin.getY(), origin.getZ());
         //            for (int j = 0; j < rawRegion.getVertices().size(); j++) {
         //                Point point = rawRegion.getVertices().get(j);
         //                System.out.println(point.getX() + "," + point.getY());
         //            }
      }
   }

   public PlanarRegionsList generatePlanarRegions(RawGPUPlanarRegionList rawGPUPlanarRegionList)
   {
      List<PlanarRegionSegmentationRawData> rawData = getSegmentationRawDataAsList(rawGPUPlanarRegionList);

//      List<PlanarRegionSegmentationRawData> rawData = new ArrayList<>();
//      getSegmentationRawData(rawGPUPlanarRegionList, rawData);

//      logSegmentationRawData(rawData);
      PlanarRegionsList planarRegionsList = updatePolygons(rawData);
//      logFinalGPUPlanarRegions(planarRegionsList);
      return planarRegionsList;
   }

   private Point3D toPoint3D(Point point)
   {
      return new Point3D(point.getX(), point.getY(), point.getZ());
   }

   public void getSegmentationRawData(RawGPUPlanarRegionList rawGPUPlanarRegionList, List<PlanarRegionSegmentationRawData> rawDataToPack)
   {

      for (int i = 0; i < rawGPUPlanarRegionList.getNumOfRegions(); i++)
      {
         RawGPUPlanarRegion region = rawGPUPlanarRegionList.getRegions().get(i);
         List<Point3D> pointCloud = region.getVertices().stream().map(this::toPoint3D).collect(Collectors.toList());
         rawDataToPack.add(new PlanarRegionSegmentationRawData(region.getId(),
                                                               new Vector3D(region.getNormal().getX(), region.getNormal().getY(), region.getNormal().getZ()),
                                                               new Point3D(region.getCentroid().getX(),
                                                                           region.getCentroid().getY(),
                                                                           region.getCentroid().getZ()),
                                                               pointCloud));
      }
   }

   public List<PlanarRegionSegmentationRawData> getSegmentationRawDataAsList(RawGPUPlanarRegionList rawGPUPlanarRegionList){
      return rawGPUPlanarRegionList.getRegions().parallelStream().map(this::convertToSegmentationRawData).collect(Collectors.toList());
   }

   public PlanarRegionSegmentationRawData convertToSegmentationRawData(RawGPUPlanarRegion rawGPUPlanarRegion)
   {

      List<Point3D> pointCloud = rawGPUPlanarRegion.getVertices().stream().map(this::toPoint3D).collect(Collectors.toList());
      PlanarRegionSegmentationRawData rawData = new PlanarRegionSegmentationRawData(rawGPUPlanarRegion.getId(),
                                                                                    new Vector3D(rawGPUPlanarRegion.getNormal().getX(),
                                                                                                 rawGPUPlanarRegion.getNormal().getY(),
                                                                                                 rawGPUPlanarRegion.getNormal().getZ()),
                                                                                    new Point3D(rawGPUPlanarRegion.getCentroid().getX(),
                                                                                                rawGPUPlanarRegion.getCentroid().getY(),
                                                                                                rawGPUPlanarRegion.getCentroid().getZ()),
                                                                                    pointCloud);
      return rawData;
   }

   private PlanarRegionsList updatePolygons(List<PlanarRegionSegmentationRawData> rawData)
   {
      if(EXPORT_SEGMENTATION_ON_EXCEPTION){
         return PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters, dataExporter);
      }else{
         return PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
      }
   }

   /*
   * Checks to see if the origin of the planar region falls inside the collision mesh of the robot. Removes the region if so.
   * */
   public PlanarRegionsList filterCollidingPlanarRegions(PlanarRegionsList regions, ScanRegionFilter filter)
   {
      int i = 0;
      while (i < regions.getNumberOfPlanarRegions())
      {
         if (!filter.test(i, regions.getPlanarRegion(i)))
            regions.pollPlanarRegion(i);
         else
            i++;
      }

      return regions;
   }
}

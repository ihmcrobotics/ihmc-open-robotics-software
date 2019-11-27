package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public class OctreeNodeSLAM
{
   private final List<RigidBodyTransform> sensorPoses = new ArrayList<>();
   private final List<Point3D[]> pointCloudMap = new ArrayList<>();
   private final List<StereoVisionPointCloudMessage> rawPointCloudMap = new ArrayList<>();

   private PlanarRegionsList planarRegionsMap;

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final CustomRegionMergeParameters customRegionMergeParameters = new CustomRegionMergeParameters();

   public static final double OCTREE_RESOLUTION = 0.02;

   public static final double MAXIMUM_DISTANCE_OF_SIMILARITY = 0.1;
   public static final double MAXIMUM_ANGLE_OF_SIMILARITY = 30.0 / 180.0 * Math.PI;

   public OctreeNodeSLAM(StereoVisionPointCloudMessage frameMessage)
   {
      rawPointCloudMap.add(frameMessage);
      pointCloudMap.add(extractPointsFromMessage(frameMessage));
      sensorPoses.add(extractSensorPoseFromMessage(frameMessage));

      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(frameMessage, OCTREE_RESOLUTION);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   public void addFrame(StereoVisionPointCloudMessage frameMessage)
   {
      Point3D[] originalPointCloud = extractPointsFromMessage(frameMessage);
      RigidBodyTransform originalSensorPose = extractSensorPoseFromMessage(frameMessage);
      RigidBodyTransform previousSensorPose = sensorPoses.get(sensorPoses.size() - 1);
      Point3D[] pointsInPreviousView = EnvironmentMappingTools.createPointsInPreviousView(originalSensorPose, previousSensorPose, originalPointCloud);
      NormalOcTree octreeInView = EnvironmentMappingTools.computeOctreeData(pointsInPreviousView, previousSensorPose.getTranslation(), OCTREE_RESOLUTION);

      List<Plane3D> validPlanes = computeValidPlanes(planarRegionsMap, octreeInView);
      if(validPlanes == null)
         return;

      RigidBodyTransform optimizedTransform = computeOptimizedSensorTransformMultiplier(planarRegionsMap, validPlanes, originalSensorPose);
      System.out.println("optimizedTransform");
      System.out.println(optimizedTransform);

      //optimizedTransform.transform(originalSensorPose);
      originalSensorPose.preMultiply(optimizedTransform);
      for (int i = 0; i < originalPointCloud.length; i++)
      {
         optimizedTransform.transform(originalPointCloud[i]);
      }
      pointCloudMap.add(originalPointCloud);
      sensorPoses.add(originalSensorPose);
      rawPointCloudMap.add(frameMessage);

      computePlanarRegionsMap();
      //computePlanarRegionsMap(originalPointCloud, originalSensorPose);
   }

   private List<Plane3D> computeValidPlanes(PlanarRegionsList planarRegionsMap, NormalOcTree octree)
   {
      int numberOfPlanarRegions = planarRegionsMap.getNumberOfPlanarRegions();
      List<Plane3D> validPlanes = new ArrayList<>();
      NormalOcTreeMessage normalOctreeMessage = OcTreeMessageConverter.convertToMessage(octree);
      UIOcTree octreeForViz = new UIOcTree(normalOctreeMessage);
      for (UIOcTreeNode uiOcTreeNode : octreeForViz)
      {
         if (!uiOcTreeNode.isNormalSet() || !uiOcTreeNode.isHitLocationSet())
            continue;

         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();

         uiOcTreeNode.getNormal(planeNormal);
         uiOcTreeNode.getHitLocation(pointOnPlane);
         Plane3D octreePlane = new Plane3D(pointOnPlane, planeNormal);

         int indexClosestPlanarRegion = -1;
         double minimumDistance = Double.MAX_VALUE;
         for (int j = 0; j < numberOfPlanarRegions; j++)
         {
            PlanarRegion planarRegion = planarRegionsMap.getPlanarRegion(j);
            Plane3D plane = planarRegion.getPlane();
            double distance = plane.distance(octreePlane.getPoint());
            if (distance < minimumDistance)
            {
               minimumDistance = distance;
               indexClosestPlanarRegion = j;
            }
         }
         double angleDistance = Math.abs(planarRegionsMap.getPlanarRegion(indexClosestPlanarRegion).getPlane().getNormal().dot(octreePlane.getNormal()));

         if (minimumDistance < MAXIMUM_DISTANCE_OF_SIMILARITY && angleDistance > Math.cos(MAXIMUM_ANGLE_OF_SIMILARITY))
         {
            validPlanes.add(octreePlane);
         }
      }

      System.out.println("octreeForViz.getNumberOfNodes() "+ octreeForViz.getNumberOfNodes());
      System.out.println("validPlanes are " + validPlanes.size());
      double ratio = (double) validPlanes.size() / octreeForViz.getNumberOfNodes();
      System.out.println("ratio " + ratio);
      if(ratio < 0.3)
         return null;

      return validPlanes;
   }

   private RigidBodyTransform computeOptimizedSensorTransformMultiplier(PlanarRegionsList planarRegionsMap, List<Plane3D> planes,
                                                                        RigidBodyTransform transformWorldToSensorPose)
   {
      double degreeLimit = 1.0;
      TDoubleArrayList initialQuery = new TDoubleArrayList();
      initialQuery.add(0.0);
      initialQuery.add(0.0);
      initialQuery.add(0.0);
      initialQuery.add(0.0);
      initialQuery.add(0.0);
      initialQuery.add(0.0);

      TDoubleArrayList lowerLimit = new TDoubleArrayList();
      lowerLimit.add(-0.05);
      lowerLimit.add(-0.05);
      lowerLimit.add(-0.05);
      lowerLimit.add(-degreeLimit / 180 * Math.PI);
      lowerLimit.add(-degreeLimit / 180 * Math.PI);
      lowerLimit.add(-degreeLimit / 180 * Math.PI);

      TDoubleArrayList upperLimit = new TDoubleArrayList();
      upperLimit.add(0.05);
      upperLimit.add(0.05);
      upperLimit.add(0.05);
      upperLimit.add(degreeLimit / 180 * Math.PI);
      upperLimit.add(degreeLimit / 180 * Math.PI);
      upperLimit.add(degreeLimit / 180 * Math.PI);

      SingleQueryFunction function = new PreMultiplierOptimizerCostFunction(planarRegionsMap, planes, transformWorldToSensorPose);
      GradientDescentModule optimizer = new GradientDescentModule(function, initialQuery);

      int maxIterations = 100;
      double convergenceThreshold = 10E-5;
      double optimizerStepSize = -0.1;
      double optimizerPerturbationSize = 0.0001;

      optimizer.setInputLowerLimit(lowerLimit);
      optimizer.setInputUpperLimit(upperLimit);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);
      optimizer.setStepSize(optimizerStepSize);
      optimizer.setPerturbationSize(optimizerPerturbationSize);

      System.out.println("initialQuery.initialQuery() " + function.getQuery(initialQuery));
      int run = optimizer.run();
      System.out.println(run);
      System.out.println("optimizer.getOptimalQuery() " + optimizer.getOptimalQuery());
      TDoubleArrayList optimalInput = optimizer.getOptimalInput();

      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      preMultiplier.setTranslation(optimalInput.get(0), optimalInput.get(1), optimalInput.get(2));
      preMultiplier.setRotationYawPitchRoll(optimalInput.get(3), optimalInput.get(4), optimalInput.get(5));

      RigidBodyTransform transformer = new RigidBodyTransform(transformWorldToSensorPose);
      transformer.multiply(preMultiplier);
      transformer.multiplyInvertOther(transformWorldToSensorPose);

      return transformer;
   }

   public void computePlanarRegionsMap(Point3D[] pointCloud, RigidBodyTransform sensorPose)
   {
      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(pointCloud, sensorPose.getTranslation(),
                                                                                                         OCTREE_RESOLUTION);

      planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                             polygonizerParameters);
   }

   public void computePlanarRegionsMap(StereoVisionPointCloudMessage frameMessage)
   {
      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(frameMessage, OCTREE_RESOLUTION);

      planarRegionsMap = EnvironmentMappingTools.buildNewMap(rawData, planarRegionsMap, customRegionMergeParameters, concaveHullFactoryParameters,
                                                             polygonizerParameters);
   }
   
   public void computePlanarRegionsMap()
   {
      List<PlanarRegionSegmentationRawData> rawData = EnvironmentMappingTools.computePlanarRegionRawData(pointCloudMap, sensorPoses, OCTREE_RESOLUTION);
      planarRegionsMap = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   public List<Point3D[]> gePointCloudMap()
   {
      return pointCloudMap;
   }

   public List<StereoVisionPointCloudMessage> getRawPointCloudMap()
   {
      return rawPointCloudMap;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsMap;
   }

   private final List<NormalOcTree> octreeMap = new ArrayList<>();

   public List<NormalOcTree> getOctreeMap()
   {
      return octreeMap;
   }

   private Point3D[] extractPointsFromMessage(StereoVisionPointCloudMessage message)
   {
      int numberOfPoints = message.getColors().size();
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = new Point3D();
         MessageTools.unpackScanPoint(message, i, pointCloud[i]);
      }
      return pointCloud;
   }

   private RigidBodyTransform extractSensorPoseFromMessage(StereoVisionPointCloudMessage message)
   {
      return new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition());
   }

   private class PreMultiplierOptimizerCostFunction implements SingleQueryFunction
   {
      PlanarRegionsList map;
      List<Plane3D> planes;
      double angleWeight = 0.5;
      double snanppingWeight = 0.05;
      RigidBodyTransform transformWorldToSensorPose;

      PreMultiplierOptimizerCostFunction(PlanarRegionsList map, List<Plane3D> planes, RigidBodyTransform transformWorldToSensorPose)
      {
         this.map = map;
         this.planes = planes;
         this.transformWorldToSensorPose = transformWorldToSensorPose;
      }

      @Override
      public double getQuery(TDoubleArrayList values)
      {
         /**
          * values : dx, dy, dz, du, dv, dw
          */
         RigidBodyTransform preMultiplier = new RigidBodyTransform();
         preMultiplier.setTranslation(values.get(0), values.get(1), values.get(2));
         preMultiplier.setRotationYawPitchRoll(values.get(3), values.get(4), values.get(5));

         RigidBodyTransform transformer = new RigidBodyTransform(transformWorldToSensorPose);
         transformer.multiply(preMultiplier);
         transformer.multiplyInvertOther(transformWorldToSensorPose);

         List<Plane3D> convertedPlanes = new ArrayList<>();
         for (int i = 0; i < planes.size(); i++)
         {
            Vector3D convertedNormal = planes.get(i).getNormalCopy();
            Point3D convertedCenter = planes.get(i).getPointCopy();

            transformer.transform(convertedNormal);
            transformer.transform(convertedCenter);
            convertedPlanes.add(new Plane3D(convertedCenter, convertedNormal));
         }

         double cost = 0;
         int numberOfPlanarRegions = map.getNumberOfPlanarRegions();
         for (int i = 0; i < convertedPlanes.size(); i++)
         {
            double minimumDistance = Double.MAX_VALUE;
            int index = -1;
            for (int j = 0; j < numberOfPlanarRegions; j++)
            {
               PlanarRegion planarRegion = planarRegionsMap.getPlanarRegion(j);
               Plane3D plane = planarRegion.getPlane();
               double distance = plane.distance(convertedPlanes.get(i).getPoint());
               if (distance < minimumDistance)
               {
                  minimumDistance = distance;
                  index = j;
               }
            }

            PlanarRegion planarRegion = planarRegionsMap.getPlanarRegion(index);
            Plane3D plane = planarRegion.getPlane();
            double angleDistance = 1 - Math.abs(plane.getNormal().dot(convertedPlanes.get(i).getNormal()));
            double centerDistance = 0;
            if(!planarRegion.isPointInsideByProjectionOntoXYPlane(convertedPlanes.get(i).getPoint()))
               centerDistance = plane.getPoint().distance(convertedPlanes.get(i).getPoint());   
            
            double distanceCost = minimumDistance + angleWeight * angleDistance + snanppingWeight * centerDistance;
            {
               cost = cost + distanceCost;
            }
         }

         return cost;
      }
   }

}

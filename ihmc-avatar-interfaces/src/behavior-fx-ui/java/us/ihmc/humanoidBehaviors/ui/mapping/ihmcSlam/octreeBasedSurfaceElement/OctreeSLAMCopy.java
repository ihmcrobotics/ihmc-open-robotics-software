package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.octreeBasedSurfaceElement;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.IhmcSLAMTools;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAMFrame;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrameOptimizerCostFunction;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.numericalMethods.GradientDescentModule;

public class OctreeSLAMCopy extends IhmcSLAM
{
   private NormalOcTree octreeNodesInPreviousView;

   private static final double MAXIMUM_DISTANCE_OF_SIMILARITY = 0.1;
   private static final double MAXIMUM_ANGLE_OF_SIMILARITY = Math.toRadians(10.0);
   
   public OctreeSLAMCopy(double octreeResolution)
   {
      super(octreeResolution);
   }

   @Override
   protected RigidBodyTransformReadOnly computeFrameCorrectionTransformer(IhmcSLAMFrame frame)
   {
      Point3DReadOnly[] pointCloudToSensor = frame.getOriginalPointCloudToSensorPose();
      RigidBodyTransformReadOnly transformWorldToSensorPose = frame.getInitialSensorPoseToWorld();

      // ComputeOctreeInPreviousView
      if (frame.isFirstFrame())
         octreeNodesInPreviousView = null;

      double[][] vertex = new double[pointCloudToSensor.length][2];

      for (int i = 0; i < pointCloudToSensor.length; i++)
      {
         vertex[i][0] = pointCloudToSensor[i].getX();
         vertex[i][1] = pointCloudToSensor[i].getY();
      }
      Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(vertex);
      ConvexPolygon2D window = new ConvexPolygon2D(supplier);

      boolean ignorePreviousOrientation = true;
      RigidBodyTransformReadOnly previousSensorPoseToWorld;
      if (ignorePreviousOrientation)
      {
         previousSensorPoseToWorld = frame.getPreviousFrame().getSensorPose();
      }
      else
      {
         previousSensorPoseToWorld = frame.getPreviousFrame().getSensorPose();
      }

      Point3D[] pointCloudToWorld = new Point3D[pointCloudToSensor.length];
      for (int i = 0; i < pointCloudToWorld.length; i++)
      {
         pointCloudToWorld[i] = new Point3D(pointCloudToSensor[i]);
         transformWorldToSensorPose.transform(pointCloudToWorld[i]);
      }
      Point3D[] convertedPointsToPreviousSensorPose = IhmcSLAMTools.createConvertedPointsToSensorPose(previousSensorPoseToWorld, pointCloudToWorld);
      boolean[] isInPreviousView = new boolean[convertedPointsToPreviousSensorPose.length];
      int numberOfPointsInPreviousView = 0;
      for (int i = 0; i < convertedPointsToPreviousSensorPose.length; i++)
      {
         Point3D point = convertedPointsToPreviousSensorPose[i];
         isInPreviousView[i] = false;
         if (window.isPointInside(point.getX(), point.getY()))
         {
            isInPreviousView[i] = true;
            numberOfPointsInPreviousView++;
         }
      }

      Point3D[] pointsInPreviousView = new Point3D[numberOfPointsInPreviousView];
      int index = 0;
      for (int i = 0; i < pointCloudToSensor.length; i++)
      {
         if (isInPreviousView[i])
         {
            pointsInPreviousView[index] = new Point3D(pointCloudToWorld[i]);
            index++;
         }
      }

      octreeNodesInPreviousView = IhmcSLAMTools.computeOctreeData(pointsInPreviousView, previousSensorPoseToWorld.getTranslation(), getOctreeResolution());
      
      // ComputeMergeableSurfaceElements
      List<IhmcSurfaceElement> mergeableSurfaceElements = new ArrayList<>();
      boolean useSufficientScoreDecision = true;
      double allowableDistanceIn2D = 0.05;

      NormalOcTree octree = octreeNodesInPreviousView;
      if (octree == null)
         return new RigidBodyTransform();

      mergeableSurfaceElements.clear();

      int numberOfPlanarRegions = planarRegionsMap.getNumberOfPlanarRegions();
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

         int indexBestPlanarRegion = -1;
         double minimumScore = Double.MAX_VALUE;
         double minimumPositionScore = Double.MAX_VALUE;
         double minimumAngleScore = Double.MAX_VALUE;
         for (int j = 0; j < numberOfPlanarRegions; j++)
         {
            PlanarRegion planarRegion = planarRegionsMap.getPlanarRegion(j);
            Plane3D plane = planarRegion.getPlane();
            double positionDistance = plane.distance(octreePlane.getPoint());
            double angleDistance = Math.abs(Math.acos(Math.abs(planarRegion.getPlane().getNormal().dot(octreePlane.getNormal()))));
            double score = positionDistance / MAXIMUM_DISTANCE_OF_SIMILARITY + angleDistance / MAXIMUM_ANGLE_OF_SIMILARITY;
            if (score < minimumScore)
            {
               minimumScore = score;
               minimumPositionScore = positionDistance;
               minimumAngleScore = angleDistance;
               indexBestPlanarRegion = j;
            }
         }

         PlanarRegion closestPlanarRegion = planarRegionsMap.getPlanarRegion(indexBestPlanarRegion);
         ConvexPolygon2D convexHull = closestPlanarRegion.getConvexHull();

         RigidBodyTransformReadOnly transformToLocal = closestPlanarRegion.getTransformToLocal();
         Point3D localPoint = new Point3D();
         transformToLocal.transform(octreePlane.getPointCopy(), localPoint);
         double distanceToPlanarRegionIn2D = convexHull.distance(new Point2D(localPoint.getX(), localPoint.getY()));
         boolean isAllowable = distanceToPlanarRegionIn2D < allowableDistanceIn2D;

         double sufficientRatio = 1.0;
         if (useSufficientScoreDecision)
            sufficientRatio = 0.5;
         if (isAllowable && minimumScore < 1.0 && minimumPositionScore < sufficientRatio && minimumAngleScore < sufficientRatio)
         {
            IhmcSurfaceElement surfaceElement = new IhmcSurfaceElement(getOctreeResolution());
            surfaceElement.setPlane(octreePlane);
            surfaceElement.setMergeablePlanarRegion(closestPlanarRegion);
            mergeableSurfaceElements.add(surfaceElement);
         }
      }
      
      if(mergeableSurfaceElements.size() == 0)
         return null;

      // Cost function
      OctreeSLAMFrameOptimizerCostFunctionCopy costFunction = new OctreeSLAMFrameOptimizerCostFunctionCopy(mergeableSurfaceElements, frame.getInitialSensorPoseToWorld());

      GradientDescentModule optimizer = new GradientDescentModule(costFunction, INITIAL_INPUT);

      int maxIterations = 100;
      double convergenceThreshold = 10E-5;
      double optimizerStepSize = -0.1;
      double optimizerPerturbationSize = 0.0001;

      optimizer.setInputLowerLimit(LOWER_LIMIT);
      optimizer.setInputUpperLimit(UPPER_LIMIT);
      optimizer.setMaximumIterations(maxIterations);
      optimizer.setConvergenceThreshold(convergenceThreshold);
      optimizer.setStepSize(optimizerStepSize);
      optimizer.setPerturbationSize(optimizerPerturbationSize);
      optimizer.setReducingStepSizeRatio(2);

      int run = optimizer.run();
      System.out.println("optimizer Query() " + run + " " + costFunction.getQuery(INITIAL_INPUT) + " " + optimizer.getOptimalQuery());
      TDoubleArrayList optimalInput = optimizer.getOptimalInput();

      RigidBodyTransform transformer = new RigidBodyTransform();
      costFunction.convertToSensorPoseMultiplier(optimalInput, transformer);

      return transformer;
   }

   class OctreeSLAMFrameOptimizerCostFunctionCopy extends SLAMFrameOptimizerCostFunction
   {
      List<IhmcSurfaceElement> surfaceElements;
      static final double POSITION_WEIGHT = 5.0;
      static final double ANGLE_WEIGHT = 5.0;
      static final double SNAPPING_PARALLEL_WEIGHT = 5.0;

      static final double LEAST_SQUARE_WEIGHT = 0.1;

      boolean assumeFlatGround = true;

      OctreeSLAMFrameOptimizerCostFunctionCopy(List<IhmcSurfaceElement> surfaceElements, RigidBodyTransformReadOnly transformWorldToSensorPose)
      {
         super(transformWorldToSensorPose);
         this.surfaceElements = surfaceElements;

         int firstPlanarRegionId = surfaceElements.get(0).getMergeablePlanarRegionId();
         for (IhmcSurfaceElement surfaceElement : surfaceElements)
         {
            if (firstPlanarRegionId != surfaceElement.getMergeablePlanarRegionId())
               assumeFlatGround = false;
         }
      }

      @Override
      public double getQuery(TDoubleArrayList values)
      {
         /**
          * values are difference in 6 dimensions : dx, dy, dz, du, dv, dw
          */
         RigidBodyTransform transformer = new RigidBodyTransform();
         convertToPointCloudTransformer(values, transformer);
         //convertToSensorPoseMultiplier(values, transformer);

         List<IhmcSurfaceElement> convertedElements = new ArrayList<>();
         for (int i = 0; i < surfaceElements.size(); i++)
         {
            Vector3D convertedNormal = surfaceElements.get(i).getPlane().getNormalCopy();
            Point3D convertedCenter = surfaceElements.get(i).getPlane().getPointCopy();

            transformer.transform(convertedNormal);
            transformer.transform(convertedCenter);
            IhmcSurfaceElement convertedElement = new IhmcSurfaceElement(surfaceElements.get(i));
            convertedElement.transform(transformer);
            convertedElements.add(convertedElement);
         }

         double cost = 0;
         int cnt = 0;
         for (int i = 0; i < convertedElements.size(); i++)
         {
            IhmcSurfaceElement convertedElement = convertedElements.get(i);
            double distance = convertedElement.getDistance(POSITION_WEIGHT, ANGLE_WEIGHT);

            double squareOfInput = 0.0;
            for (double value : values.toArray())
            {
               squareOfInput = squareOfInput + value * value;
            }

            double distanceCost = distance + LEAST_SQUARE_WEIGHT * squareOfInput;
            {
               cost = cost + distanceCost;
            }
            if (convertedElement.isInPlanarRegion())
               cnt++;
         }

         double snappingScore;
         if (assumeFlatGround)
            snappingScore = 0.0;
         else
            snappingScore = 1 - (double) cnt / convertedElements.size();
         cost = cost + snappingScore * SNAPPING_PARALLEL_WEIGHT;

         return cost;
      }
   }
}

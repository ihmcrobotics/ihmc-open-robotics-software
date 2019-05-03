package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import controller_msgs.msg.dds.DoorParameterPacket;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.ros2.Ros2Node;

public class DoorParameterCalculator extends AbstractObjectParameterCalculator<DoorParameterPacket>
{
   private static final int numberOfTimesToRANSAC = 100;
   private static final double thresholdOfInlier = 0.05;
   private final List<Point3DBasics> pointsInlier = new ArrayList<>();
   private final List<Point3DBasics> tempOutlierPoints = new ArrayList<>();

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

   private static final int numberOfSearchingRectangle = 20;

   private final Map<DoorVertexName, Point3D> doorVerticesInWorld = new HashMap<>();
   private final Map<DoorVertexName, Point3D> doorVerticesInPCA = new HashMap<>();

   private enum DoorVertexName
   {
      TOP_RIGHT, TOP_LEFT, BOTTOM_LEFT, BOTTOM_RIGHT;
      DoorVertexName nextName()
      {
         switch (this)
         {
         case TOP_RIGHT:
            return TOP_LEFT;
         case TOP_LEFT:
            return BOTTOM_LEFT;
         case BOTTOM_LEFT:
            return BOTTOM_RIGHT;
         case BOTTOM_RIGHT:
            return TOP_RIGHT;
         default:
            return null;
         }
      }
   }

   public DoorParameterCalculator(Ros2Node ros2Node, Class<DoorParameterPacket> packetType)
   {
      super(ros2Node, packetType);
   }

   /**
    * ROI(Door), ROI(Door Handle)
    */
   @Override
   public void calculate(RegionOfInterest... additionalROIs)
   {
      ransac();
      findPrincipalComponent();
      findRectangle();
      assignHingedPoint(additionalROIs[0]);
   }

   private void ransac()
   {
      Random random = new Random(0612L);
      int numberOfPointInROI = pointCloudToCalculate.size();

      int maximumNumberOfInlier = 0;
      PlaneEquation bestPlane = new PlaneEquation();

      for (int i = 0; i < numberOfTimesToRANSAC; i++)
      {
         int index1 = -1, index2 = -1, index3 = -1;
         index1 = random.nextInt(numberOfPointInROI);
         while (index1 == index2 || index2 == index3 || index3 == index1)
         {
            index2 = random.nextInt(numberOfPointInROI);
            index3 = random.nextInt(numberOfPointInROI);
         }

         PlaneEquation selectedPlane = new PlaneEquation(pointCloudToCalculate.get(index1), pointCloudToCalculate.get(index2),
                                                         pointCloudToCalculate.get(index3));

         int numberOfInlier = 0;
         for (int j = 0; j < numberOfPointInROI; j++)
         {
            Point3DBasics point = pointCloudToCalculate.get(j);
            double distance = selectedPlane.distance(point);
            if (distance <= thresholdOfInlier)
               numberOfInlier++;
         }

         if (numberOfInlier > maximumNumberOfInlier)
         {
            bestPlane.set(selectedPlane);
            maximumNumberOfInlier = numberOfInlier;
         }
      }

      pointsInlier.clear();
      for (int i = 0; i < numberOfPointInROI; i++)
      {
         Point3DBasics point = pointCloudToCalculate.get(i);
         double distance = bestPlane.distance(point);
         if (distance <= thresholdOfInlier)
            pointsInlier.add(point);
      }

      LogTools.info("Best Plane is " + bestPlane.getInformation());
      LogTools.info("Number of points to be fitted is " + pointsInlier.size());

      // TODO: remove ----------------------
      tempOutlierPoints.clear();
      for (int i = 0; i < numberOfPointInROI; i++)
      {
         Point3DBasics point = pointCloudToCalculate.get(i);
         double distance = bestPlane.distance(point);
         if (distance > thresholdOfInlier)
            tempOutlierPoints.add(point);
      }

      Vector3D total = new Vector3D();
      for (Point3DBasics outPoint : tempOutlierPoints)
         total.add(outPoint);
      total.setX(total.getX() / tempOutlierPoints.size());
      total.setY(total.getY() / tempOutlierPoints.size());
      total.setZ(total.getZ() / tempOutlierPoints.size());

      LogTools.info("Number of points that outside of door is " + tempOutlierPoints.size() + ", X: " + total.getX() + ", Y: " + total.getY() + ", Z: "
            + total.getZ());
      // TODO: remove ----------------------
   }

   private void findPrincipalComponent()
   {
      pca.clear();
      pca.addAllDataPoints(pointsInlier);
      pca.compute();
   }

   private void findRectangle()
   {
      Point3D centerPosition = new Point3D();
      pca.getMean(centerPosition);
      Vector3D normal = new Vector3D();
      pca.getThirdVector(normal);
      PlaneEquation clusteredPlane = new PlaneEquation(normal, centerPosition);

      LogTools.info("assumed center is " + centerPosition);
      FiniteRectangleCalculator finiteRectangleCalculator = new FiniteRectangleCalculator(clusteredPlane, centerPosition);
      finiteRectangleCalculator.projectPointsOnPlane(pointsInlier);

      RotationMatrix bestPCARotationMatrix = new RotationMatrix();
      RotationMatrix searchingPCARotationMatrix = new RotationMatrix();
      pca.getPrincipalFrameRotationMatrix(searchingPCARotationMatrix);
      double minimumArea = Double.POSITIVE_INFINITY;
      for (int i = 0; i < numberOfSearchingRectangle; i++)
      {
         finiteRectangleCalculator.compute(searchingPCARotationMatrix);
         double area = finiteRectangleCalculator.area();
         if (area < minimumArea)
         {
            minimumArea = area;
            bestPCARotationMatrix.set(searchingPCARotationMatrix);
            LogTools.info("minimum area is " + minimumArea);
         }

         double rotatingAngle = Math.PI / 2.0 / numberOfSearchingRectangle;
         searchingPCARotationMatrix.appendYawRotation(rotatingAngle);
      }
      finiteRectangleCalculator.compute(bestPCARotationMatrix);
      LogTools.info("final door area is " + finiteRectangleCalculator.area());

      for (DoorVertexName vertexName : DoorVertexName.values())
      {
         LogTools.info("doorVerticesInPCA vertexName " + vertexName + " " + doorVerticesInPCA.get(vertexName));
      }
   }

   private void assignHingedPoint(RegionOfInterest handleROI)
   {
      double assumedDoorCenterHeight = 0.0;
      for (DoorVertexName vertexName : DoorVertexName.values())
         assumedDoorCenterHeight = +doorVerticesInPCA.get(vertexName).getZ();
      assumedDoorCenterHeight = assumedDoorCenterHeight / DoorVertexName.values().length;

      for (DoorVertexName vertexName : DoorVertexName.values())
      {
         double vertexHeight = doorVerticesInPCA.get(vertexName).getZ();
         double nextVertexHeight = doorVerticesInPCA.get(vertexName.nextName()).getZ();
         if (vertexHeight > assumedDoorCenterHeight && nextVertexHeight > assumedDoorCenterHeight)
         {
            DoorVertexName vertexInPCA = vertexName;
            for (DoorVertexName vertexInWorld : DoorVertexName.values())
            {
               doorVerticesInWorld.put(vertexInWorld, doorVerticesInPCA.get(vertexInPCA));
               vertexInPCA = vertexInPCA.nextName();
            }
         }
      }

      for (DoorVertexName vertexName : DoorVertexName.values())
      {
         LogTools.info("doorVerticesInWorld vertexName " + vertexName + " " + doorVerticesInWorld.get(vertexName));
      }

      newPacket.get().setDoorHeight(doorVerticesInWorld.get(DoorVertexName.BOTTOM_LEFT).getZ());
      if (handleROI == null)
      {
         newPacket.get().getHingedPointOnGround().set(doorVerticesInWorld.get(DoorVertexName.BOTTOM_LEFT));
         newPacket.get().getEndPointOnGround().set(doorVerticesInWorld.get(DoorVertexName.BOTTOM_RIGHT));
      }
      else
      {
         double marginXLeft = handleROI.getXOffset() - objectROI.getXOffset();
         double marginXRight = objectROI.getXOffset() + objectROI.getWidth() - handleROI.getXOffset() - handleROI.getWidth();
         if (marginXLeft < 0 || marginXRight < 0)
         {
            LogTools.warn("The detected handle roi is not in door roi.");
         }

         if (marginXLeft < marginXRight)
         {
            newPacket.get().getHingedPointOnGround().set(doorVerticesInWorld.get(DoorVertexName.BOTTOM_LEFT));
            newPacket.get().getEndPointOnGround().set(doorVerticesInWorld.get(DoorVertexName.BOTTOM_RIGHT));
         }
         else
         {
            newPacket.get().getHingedPointOnGround().set(doorVerticesInWorld.get(DoorVertexName.BOTTOM_RIGHT));
            newPacket.get().getEndPointOnGround().set(doorVerticesInWorld.get(DoorVertexName.BOTTOM_LEFT));
         }
      }
   }

   /**
    * Standard plane equation form.
    * a*x + b*y + c*z + d = 0.
    */
   private class PlaneEquation
   {
      private final Vector3D normalVector = new Vector3D();
      private double constantD = 0.0;

      private PlaneEquation()
      {

      }

      private PlaneEquation(Vector3DBasics normal, double constant)
      {
         normalVector.set(normal);
         constantD = constant;
      }

      private PlaneEquation(Vector3DBasics normal, Point3DBasics pointOnPlane)
      {
         normalVector.set(normal);
         Vector3D pointVector = new Vector3D(pointOnPlane);
         constantD = -normalVector.dot(pointVector);
      }

      private PlaneEquation(Point3DBasics point1, Point3DBasics point2, Point3DBasics point3)
      {
         Vector3D vector1To2 = new Vector3D(point2);
         vector1To2.sub(point1);
         Vector3D vector1To3 = new Vector3D(point3);
         vector1To3.sub(point1);

         normalVector.cross(vector1To2, vector1To3);
         Vector3D pointVector = new Vector3D(point1);
         constantD = -normalVector.dot(pointVector);
      }

      private void set(PlaneEquation other)
      {
         normalVector.set(other.normalVector);
         constantD = other.constantD;
      }

      private double distance(Point3DBasics point)
      {
         Vector3D pointVector = new Vector3D(point);
         return Math.abs(normalVector.dot(pointVector) + constantD) / Math.sqrt(normalVector.lengthSquared());
      }

      private void project(Point3DBasics point, Point3DBasics pointToPack)
      {
         Vector3D pointVector = new Vector3D(point);
         double projectionConstant = -(normalVector.dot(pointVector) + constantD) / normalVector.lengthSquared();
         pointToPack.setX(point.getX() + projectionConstant * normalVector.getX());
         pointToPack.setY(point.getY() + projectionConstant * normalVector.getY());
         pointToPack.setZ(point.getZ() + projectionConstant * normalVector.getZ());
      }

      private String getInformation()
      {
         return "A: " + normalVector.getX() + ", B: " + normalVector.getY() + ", C: " + normalVector.getZ() + ", D: " + constantD;
      }
   }

   private class FiniteRectangleCalculator
   {
      private final PlaneEquation planeDefinition = new PlaneEquation();
      private final Point3D centerDefinition = new Point3D();
      private final List<Point3DBasics> projectedPoints = new ArrayList<>();

      private double area = 0.0;

      private FiniteRectangleCalculator(PlaneEquation clusteredPlane, Point3DBasics centerPosition)
      {
         planeDefinition.set(clusteredPlane);
         centerDefinition.set(centerPosition);
      }

      private void projectPointsOnPlane(List<Point3DBasics> points)
      {
         projectedPoints.clear();
         for (Point3DBasics point : points)
         {
            Point3D projectedPoint = new Point3D();
            planeDefinition.project(point, projectedPoint);
            projectedPoints.add(projectedPoint);
         }
      }

      private void compute(RotationMatrix clusteredRotationMatrix)
      {
         RigidBodyTransform transformer = new RigidBodyTransform(clusteredRotationMatrix, centerDefinition);
         List<Point2D> localPoints = new ArrayList<Point2D>();
         for (Point3DBasics point : projectedPoints)
         {
            Point3D transformedPoint = new Point3D();
            transformer.inverseTransform(point, transformedPoint);

            if (Math.abs(transformedPoint.getZ()) >= 0.001)
               LogTools.warn("The projection didn't work properly !!!!! ");

            Point2D convertedPoint = new Point2D(transformedPoint.getX(), transformedPoint.getY());
            localPoints.add(convertedPoint);
         }

         double positiveXLength = 0.0, negativeXLength = 0.0, positiveYLength = 0.0, negativeYLength = 0.0;
         for (int i = 0; i < localPoints.size(); i++)
         {
            Point2D point = localPoints.get(i);
            if (point.getX() > positiveXLength)
               positiveXLength = point.getX();

            if (point.getX() < negativeXLength)
               negativeXLength = point.getX();

            if (point.getY() > positiveYLength)
               positiveYLength = point.getY();

            if (point.getY() < negativeYLength)
               negativeYLength = point.getY();
         }

         area = (positiveXLength - negativeXLength) * (positiveYLength - negativeYLength);

         submitVertex(DoorVertexName.TOP_RIGHT, transformer, positiveXLength, positiveYLength);
         submitVertex(DoorVertexName.TOP_LEFT, transformer, negativeXLength, positiveYLength);
         submitVertex(DoorVertexName.BOTTOM_LEFT, transformer, negativeXLength, negativeYLength);
         submitVertex(DoorVertexName.BOTTOM_RIGHT, transformer, positiveXLength, negativeYLength);
      }

      private void submitVertex(DoorVertexName vertexName, RigidBodyTransform pcaTransform, double xInPCA, double yInPCA)
      {
         RigidBodyTransform vertexInWorld = new RigidBodyTransform(pcaTransform);
         vertexInWorld.appendTranslation(xInPCA, yInPCA, 0.0);
         doorVerticesInPCA.put(vertexName, new Point3D(vertexInWorld.getTranslationVector()));
      }

      private double area()
      {
         return area;
      }
   }
}
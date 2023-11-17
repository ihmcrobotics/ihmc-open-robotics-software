package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class CliffHeightMapCostExampleCode
{
   // This value is squared to get the area of the grip point, each grid point is this distance in centimeters
   private final double gridResolution = 0.05;
   // Total width of the height map in meters
   private final double gridSize = 1.5;

   public CliffHeightMapCostExampleCode()
   {
      HeightMapData heightMapData = new HeightMapData(gridResolution, gridSize, 0.0, 0.0);

      // Sets up a plane slanted to the ground
      Plane3D plane = new Plane3D();
      plane.getPoint().set(0.5, 0.5, 0.5);
      plane.getNormal().set(Axis3D.Z);
      new AxisAngle(0.707, 0.707, 0.0, Math.toRadians(10.0)).transform(plane.getNormal());
      System.out.println("Plane normal:" + plane.getNormal());

      // Sets the height map to be at the same heights as the plane
      setHeightMapToPlane(heightMapData, plane);

      DiscreteFootstep footstep = new DiscreteFootstep(-9, 13, 10, RobotSide.LEFT);
      System.out.println("Footstep location (x) = " + footstep.getX());
      System.out.println("Footstep location (y) = " + footstep.getY());

      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.5, 0.6);
      footPolygon.addVertex(0.5, -0.6);
      footPolygon.addVertex(-0.5, 0.6);
      footPolygon.addVertex(-0.5, -0.6);
      footPolygon.update();

      ConvexPolygon2D candidateFootPolygon = new ConvexPolygon2D();
      DiscreteFootstepTools.getFootPolygon(footstep, footPolygon, candidateFootPolygon);

      HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();
      RigidBodyTransform snapTransform = heightMapSnapper.snapPolygonToHeightMap(candidateFootPolygon, heightMapData);
      System.out.println("Snap transform:\n" + snapTransform);

      RigidBodyTransform snappedStepTransform = new RigidBodyTransform();
      DiscreteFootstepTools.getSnappedStepTransform(footstep, snapTransform, snappedStepTransform);

      Pose3D snappedStepPose = new Pose3D();
      snappedStepPose.set(snappedStepTransform);
      System.out.println("Snapped step pose:\n" + snappedStepPose);

      candidateFootPolygon.applyTransform(snapTransform, false);
      System.out.println("Transformed polygon centroid");
      System.out.println(candidateFootPolygon.getCentroid());
      System.out.println(candidateFootPolygon);

      Plane3D bestFitPlane = new Plane3D();
      bestFitPlane.getPoint().set(snappedStepTransform.getTranslation());
      bestFitPlane.getNormal().set(Axis3D.Z);
      snappedStepTransform.getRotation().transform(bestFitPlane.getNormal());

      System.out.println("Plane height at 0.5, 0.5 = " + bestFitPlane.getZOnPlane(0.5, 0.5));
      System.out.println("Plane normal: " + bestFitPlane.getNormal());
   }

   private void setHeightMapToPlane(HeightMapData heightMapData, Plane3D plane)
   {
      double epsilon = 1e-8;
      double halfWidth = 0.5 * (gridSize + gridResolution) - epsilon;
      double minX = - halfWidth;
      double maxX = halfWidth;
      double minY = - halfWidth;
      double maxY = halfWidth;

      for (double xi = minX; xi <= maxX; xi += gridResolution)
      {
         for (double yi = minY; yi <= maxY; yi += gridResolution)
         {
            heightMapData.setHeightAt(xi, yi, plane.getZOnPlane(xi, yi));
         }
      }
   }

   public static void main(String[] args)
   {
      new CliffHeightMapCostExampleCode();
   }
}

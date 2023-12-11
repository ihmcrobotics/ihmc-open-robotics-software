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
   public CliffHeightMapCostExampleCode()
   {
      double gridResolution = 0.05;
      double gridSize = 1.5;

      HeightMapData heightMapData = new HeightMapData(gridResolution, gridSize, 0.0, 0.0);
      Plane3D plane = new Plane3D();
      plane.getPoint().set(0.5, 0.5, 0.5);
      plane.getNormal().set(Axis3D.Z);
      new AxisAngle(0.707, 0.707, 0.0, Math.toRadians(10.0)).transform(plane.getNormal());
      System.out.println("Plane normal:" + plane.getNormal());

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

      DiscreteFootstep footstep = new DiscreteFootstep(-9, 13, 10, RobotSide.LEFT);
      System.out.println("Step x = " + footstep.getX());
      System.out.println("Step y = " + footstep.getY());

      HeightMapPolygonSnapper heightMapSnapper = new HeightMapPolygonSnapper();

      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(0.5, 0.6);
      footPolygon.addVertex(0.5, -0.6);
      footPolygon.addVertex(-0.5, 0.6);
      footPolygon.addVertex(-0.5, -0.6);
      footPolygon.update();

      ConvexPolygon2D candidateFootPolygon = new ConvexPolygon2D();
      DiscreteFootstepTools.getFootPolygon(footstep, footPolygon, candidateFootPolygon);

      RigidBodyTransform snapTransform = heightMapSnapper.snapPolygonToHeightMap(candidateFootPolygon, heightMapData);
      System.out.println("snap transform");
      System.out.println(snapTransform);

      RigidBodyTransform snappedStepTransform = new RigidBodyTransform();
      DiscreteFootstepTools.getSnappedStepTransform(footstep, snapTransform, snappedStepTransform);

      Pose3D snappedStepPose = new Pose3D();
      snappedStepPose.set(snappedStepTransform);
      System.out.println("Snapped step pose");
      System.out.println(snappedStepPose);

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

   public static void main(String[] args)
   {
      new CliffHeightMapCostExampleCode();
   }
}

package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.depthData.CollisionShapeTester;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullPruningFilteringTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class PerceptionFilterTools
{
   public static void filterCollidingPlanarRegions(FramePlanarRegionsList regionsInSensor, CollidingScanRegionFilter filter)
   {
      int i = 0;
      while (i < regionsInSensor.getPlanarRegionsList().getNumberOfPlanarRegions())
      {
         PlanarRegion regionInWorld = regionsInSensor.getPlanarRegionsList().getPlanarRegion(i).copy();
         regionInWorld.applyTransform(regionsInSensor.getSensorToWorldFrameTransform());
         boolean collision = !filter.test(i, regionInWorld);
         if (collision)
         {
            regionsInSensor.getPlanarRegionsList().pollPlanarRegion(i);
         }
         else
            i++;
      }
   }

   public static void applyCollisionFilter(FramePlanarRegionsList framePlanarRegionsList, CollidingScanRegionFilter collisionFilter)
   {
      collisionFilter.update();
      PlanarRegionsList planarRegionsList = framePlanarRegionsList.getPlanarRegionsList();

      List<PlanarRegion> filteredPlanarRegions = planarRegionsList.getPlanarRegionsAsList().parallelStream()
                                                                  .filter(region -> {
                                                                     PlanarRegion regionInWorld = region.copy();
                                                                     regionInWorld.applyTransform(framePlanarRegionsList.getSensorToWorldFrameTransform());
                                                                     return collisionFilter.test(0, regionInWorld);
                                                                  }).toList();

      framePlanarRegionsList.getPlanarRegionsList().clear();
      framePlanarRegionsList.getPlanarRegionsList().addPlanarRegions(filteredPlanarRegions);
   }

   public static CollidingScanRegionFilter createHumanoidShinCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      CollisionShapeTester shapeTester = new CollisionShapeTester(fullRobotModel, collisionBoxProvider);
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
      return new CollidingScanRegionFilter(shapeTester);
   }

   public static void applyBoundingBoxFilter(PlanarRegionsList planarRegionsList, BoundingBox3D boundingBox)
   {
      PlanarRegionsList result = null;

      Plane3D plane = new Plane3D(new Point3D(boundingBox.getMinX(), 0, 0), new Vector3D(1, 0, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, planarRegionsList);

      plane = new Plane3D(new Point3D(boundingBox.getMaxX(), 0, 0), new Vector3D(-1, 0, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, result);

      plane = new Plane3D(new Point3D(0, boundingBox.getMinY(), 0), new Vector3D(0, 1, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, result);

      plane = new Plane3D(new Point3D(0, boundingBox.getMaxY(), 0), new Vector3D(0, -1, 0));
      result = PlanarRegionCuttingTools.cutByPlane(plane, result);

      planarRegionsList.clear();
      planarRegionsList.addPlanarRegions(result.getPlanarRegionsAsList());
   }

   public static void filterShadowRegions(FramePlanarRegionsList framePlanarRegionsList, double maxAngleFromNormal)
   {
      int i = 0;
      double angleFromNormal = Math.toRadians(maxAngleFromNormal);
      double minDot = Math.cos(Math.PI / 2.0 - angleFromNormal);
      while (i < framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions())
      {
         PlanarRegion regionInSensorFrame = framePlanarRegionsList.getPlanarRegionsList().getPlanarRegion(i);
         Vector3D vectorToRegion = new Vector3D(regionInSensorFrame.getPoint());
         vectorToRegion.normalize();

         if (Math.abs(vectorToRegion.dot(regionInSensorFrame.getNormal())) < minDot)
         {
            framePlanarRegionsList.getPlanarRegionsList().getPlanarRegionsAsList().remove(i);
         }
         else
         {
            i++;
         }
      }
   }

   public static void applyConcaveHullFilters(PlanarRegionsList planarRegionsList, PolygonizerParameters polygonizerParameters)
   {
      for (PlanarRegion region : planarRegionsList.getPlanarRegionsAsList())
      {
         ConcaveHull concaveHull = new ConcaveHull(region.getConcaveHull());

         // Apply some simple filtering to reduce the number of vertices and hopefully the number of convex polygons.
         double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
         double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
         double lengthThreshold = polygonizerParameters.getLengthThreshold();

         ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHull);
         ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHull);

         double depthThreshold = polygonizerParameters.getDepthThreshold();
         List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
         ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull, depthThreshold, decomposedPolygons);

         // Pack the data in PlanarRegion
         PlanarRegion filteredRegion = new PlanarRegion(region.getTransformToWorld(), concaveHull.getConcaveHullVertices(), decomposedPolygons);
         filteredRegion.setRegionId(region.getRegionId());

         region.set(filteredRegion);

         //if (polygonizerParameters.getCutNarrowPassage())
         //    ConcaveHull hull = ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(lengthThreshold, concaveHull).getConcaveHulls().stream().toList().get(0);
         //else
         //   return concaveHulls;
      }
   }
}

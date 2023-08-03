package us.ihmc.perception.geometry;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PlanarRegionCuttingTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PlanarRegionMapTest
{
   @Test
   public void testPlanarRegionCutting()
   {
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);
      Point2D pointE = new Point2D(0.5, 0.0);
      Point2D pointF = new Point2D(0.5, 1.0);

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(pointA.getX(), pointA.getY());
      convexPolygon.addVertex(pointB.getX(), pointB.getY());
      convexPolygon.addVertex(pointC.getX(), pointC.getY());
      convexPolygon.addVertex(pointD.getX(), pointD.getY());
      convexPolygon.update();

      ConvexPolygon2D expectedPolygon = new ConvexPolygon2D();
      expectedPolygon.addVertex(pointE.getX(), pointE.getY());
      expectedPolygon.addVertex(pointB.getX(), pointB.getY());
      expectedPolygon.addVertex(pointC.getX(), pointC.getY());
      expectedPolygon.addVertex(pointF.getX(), pointF.getY());
      expectedPolygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, -Math.PI / 2, 0.0), new Point3D()), convexPolygon);
      PlanarRegion expectedRegion = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, -Math.PI / 2, 0.0), new Point3D()), expectedPolygon);

      Plane3D plane = new Plane3D(0.0, 0.0, 0.5, 0.0, 0.0, 1.0);

      List<PlanarRegion> cuttingResultRegions = PlanarRegionCuttingTools.cutRegionByPlane(plane, planarRegion);

      LogTools.info("Number of planar regions: " + cuttingResultRegions.size());

      PlanarRegion resultRegion = cuttingResultRegions.get(0);

      for (int i = 0; i < resultRegion.getConcaveHullSize(); i++)
      {
         LogTools.info("Point " + i + ": " + resultRegion.getConcaveHullPoint3DInWorld(i));
      }

      PlanarRegionTestTools.assertPlanarRegionsGeometricallyEqual(expectedRegion, resultRegion, 1e-7);
   }

   // Test for chopping off extra parts given two test regions as above
   @Disabled
   @Test
   public void testPlanarRegionCutting2()
   {
      // For first region (actual and expected)
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);
      Point2D pointE = new Point2D(0.5, 0.0);
      Point2D pointF = new Point2D(0.5, 1.0);

      // For second region (actual and expected)
      Point2D pointK = new Point2D(0.0, 0.0);
      Point2D pointH = new Point2D(0.5, 0.0);
      Point2D pointI = new Point2D(0.5, 1.0);
      Point2D pointL = new Point2D(0.0, 1.0);


      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(pointA.getX(), pointA.getY());
      convexPolygon.addVertex(pointB.getX(), pointB.getY());
      convexPolygon.addVertex(pointC.getX(), pointC.getY());
      convexPolygon.addVertex(pointD.getX(), pointD.getY());
      convexPolygon.update();

      PlanarRegion planarRegionOne = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, -Math.PI / 2, 0.0), new Point3D(0.5, 0.0, -0.5)), convexPolygon);
      PlanarRegion planarRegionTwo = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 0.0)), convexPolygon);

      // Expected region one after cut
      ConvexPolygon2D expectedPolygonOne = new ConvexPolygon2D();
      expectedPolygonOne.addVertex(pointE.getX(), pointE.getY());
      expectedPolygonOne.addVertex(pointB.getX(), pointB.getY());
      expectedPolygonOne.addVertex(pointC.getX(), pointC.getY());
      expectedPolygonOne.addVertex(pointF.getX(), pointF.getY());
      expectedPolygonOne.update();

      PlanarRegion expectedRegionOne = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, -Math.PI / 2, 0.0), new Point3D(0.5, 0.0, -0.5)), expectedPolygonOne);

      // Expected region two after cut
      ConvexPolygon2D expectedPolygonTwo = new ConvexPolygon2D();
      expectedPolygonTwo.addVertex(pointK.getX(), pointK.getY());
      expectedPolygonTwo.addVertex(pointH.getX(), pointH.getY());
      expectedPolygonTwo.addVertex(pointI.getX(), pointI.getY());
      expectedPolygonTwo.addVertex(pointL.getX(), pointL.getY());
      expectedPolygonTwo.update();

      PlanarRegion expectedRegionTwo = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 0.0)), expectedPolygonTwo);

      List<PlanarRegion> planarRegions = PlanarRegionCuttingTools.chopOffExtraPartsAtIntersection(planarRegionOne, planarRegionTwo);

      PlanarRegion resultRegionOne = planarRegions.get(0);
      PlanarRegion resultRegionTwo = planarRegions.get(1);

      for (int i = 0; i < resultRegionOne.getConcaveHullSize(); i++)
      {
         LogTools.info("[Result One] Point " + i + ": " + resultRegionOne.getConcaveHullPoint3DInWorld(i));
      }

      for (int i = 0; i < resultRegionTwo.getConcaveHullSize(); i++)
      {
         LogTools.info("[Result Two] Point " + i + ": " + resultRegionTwo.getConcaveHullPoint3DInWorld(i));
      }

      PlanarRegionTestTools.assertPlanarRegionsGeometricallyEqual(expectedRegionOne, resultRegionOne, 1e-7);
      PlanarRegionTestTools.assertPlanarRegionsGeometricallyEqual(expectedRegionTwo, resultRegionTwo, 1e-7);
   }

   // Test for factor graph based optimization of planar regions
   @Test
   public void testPlanarRegionOptimization()
   {
      // For first region (actual and expected)
      Point2D pointA = new Point2D(0.0, 0.0);
      Point2D pointB = new Point2D(1.0, 0.0);
      Point2D pointC = new Point2D(1.0, 1.0);
      Point2D pointD = new Point2D(0.0, 1.0);

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(pointA.getX(), pointA.getY());
      convexPolygon.addVertex(pointB.getX(), pointB.getY());
      convexPolygon.addVertex(pointC.getX(), pointC.getY());
      convexPolygon.addVertex(pointD.getX(), pointD.getY());
      convexPolygon.update();


      PlanarRegion planarRegionOne = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, -Math.PI / 2, 0.0), new Point3D(0.0, 0.0, 0.0)), convexPolygon);
      PlanarRegion planarRegionTwo = new PlanarRegion(new RigidBodyTransform(new Quaternion(0.0, 0.0, 0.0), new Point3D(-1.0, 0.0, 0.0)), convexPolygon);
      PlanarRegion planarRegionThree = new PlanarRegion(new RigidBodyTransform(new Quaternion(-Math.PI / 2, -Math.PI / 2, 0.0), new Point3D(-1.0, 0.0, 0.0)),
                                                        convexPolygon);

      RigidBodyTransform sensorToWorldOne = new RigidBodyTransform(new Quaternion(-Math.PI/4, 0.0, 0.0), new Point3D(-1.0, 1.0, 0.5));
      RigidBodyTransform sensorToWorldTwo = new RigidBodyTransform(new Quaternion(-Math.PI/4, 0.0, 0.0), new Point3D(-0.9, 0.9, 0.5));
      RigidBodyTransform sensorToWorldThree = new RigidBodyTransform(new Quaternion(-Math.PI/4, 0.0, 0.0), new Point3D(-0.8, 0.8, 0.5));

      PlanarRegionsList regionsList = new PlanarRegionsList();
      regionsList.addPlanarRegion(planarRegionOne);
      regionsList.addPlanarRegion(planarRegionTwo);
      regionsList.addPlanarRegion(planarRegionThree);

      PlanarRegionsList planarRegionsListOne = regionsList.copy();
      RigidBodyTransform worldToSensorOne = new RigidBodyTransform(sensorToWorldOne);
      worldToSensorOne.invert();
      planarRegionsListOne.applyTransform(worldToSensorOne);

      PlanarRegionsList planarRegionsListTwo = regionsList.copy();
      RigidBodyTransform worldToSensorTwo = new RigidBodyTransform(sensorToWorldTwo);
      worldToSensorTwo.invert();
      planarRegionsListTwo.applyTransform(worldToSensorTwo);

      PlanarRegionsList planarRegionsListThree = regionsList.copy();
      RigidBodyTransform worldToSensorThree = new RigidBodyTransform(sensorToWorldThree);
      worldToSensorThree.invert();
      planarRegionsListThree.applyTransform(worldToSensorThree);

      PerceptionDebugTools.printPlanarRegionsListVertices("List One", planarRegionsListOne, true);
      PerceptionDebugTools.printPlanarRegionsListVertices("List Two", planarRegionsListTwo, true);
      PerceptionDebugTools.printPlanarRegionsListVertices("List Three", planarRegionsListThree, true);

      PlanarRegionMap planarRegionMap = new PlanarRegionMap(true);

      planarRegionMap.setInitialSensorPose(sensorToWorldOne);
      planarRegionMap.registerRegions(planarRegionsListOne, sensorToWorldOne, null);

      RigidBodyTransform keyframePoseThree = planarRegionMap.registerRegions(planarRegionsListTwo, sensorToWorldTwo, null);
      PerceptionDebugTools.printTransform("Keyframe Pose Two", keyframePoseThree, true);

      RigidBodyTransform keyframePoseTwo = planarRegionMap.registerRegions(planarRegionsListThree, sensorToWorldThree, null);
      PerceptionDebugTools.printTransform("Keyframe Pose Three", keyframePoseTwo, true);

      PerceptionDebugTools.printPlanarRegionsListVertices("Final Map", planarRegionMap.getMapRegions(), true);
   }
}

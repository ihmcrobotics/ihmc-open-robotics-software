package us.ihmc.pathPlanning.bodyPathPlanner;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class BodyPathPlannerToolsTest
{
   @Test
   public void testMoveAlongPoint()
   {
      List<Pose3DReadOnly> bodyPath = new ArrayList<>();
      Point3D point0 = new Point3D(0.2, 0.0, 0.0);
      Point3D point1 = new Point3D(0.5, 0.0, 0.0);
      Point3D point2 = new Point3D(10.0, 0.0, 0.0);
      Point3D point3 = new Point3D(11.0, 0.0, 0.0);

      Pose3DReadOnly pose0 = new Pose3D(point0, new Quaternion());
      Pose3DReadOnly pose1 = new Pose3D(point1, new Quaternion());
      Pose3DReadOnly pose2 = new Pose3D(point2, new Quaternion());
      Pose3DReadOnly pose3 = new Pose3D(point3, new Quaternion());

      bodyPath.add(pose0);
      bodyPath.add(pose1);
      bodyPath.add(pose2);
      bodyPath.add(pose3);

      // Test if we start at the beginning of the plan
      {
         Point3D startPoint = new Point3D(0.2, -0.5, 0.0);
         Point3D movedPoint = new Point3D(0.0, 0.0, 0.0);

         // start 1 meter in
         double distance = 1.0;
         int segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         Point3D expectedMovedPoint = new Point3D(startPoint);
         expectedMovedPoint.setY(0.0);
         expectedMovedPoint.addX(distance);

         assertEquals(1, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // start at beginning
         distance = 0.0;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(startPoint);
         expectedMovedPoint.setY(0.0);
         expectedMovedPoint.addX(distance);

         assertEquals(0, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // test far in
         distance = 10.5;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(startPoint);
         expectedMovedPoint.setY(0.0);
         expectedMovedPoint.addX(distance);

         assertEquals(2, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // test at end
         distance = 10.8;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(startPoint);
         expectedMovedPoint.setY(0.0);
         expectedMovedPoint.addX(distance);

         assertEquals(2, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // test past end
         distance = 14.0;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(point3);

         assertEquals(2, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);
      }


      // TODO Test if we start halfway through the plan
      // TODO Test if we start before the plan

      // Test if we start at the beginning of the plan
      {
         Point3D startPoint = new Point3D(0.0, -0.5, 0.0);
         Point3D movedPoint = new Point3D(0.0, 0.0, 0.0);

         // start 1 meter in
         double distance = 1.0;
         int segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         Point3D expectedMovedPoint = new Point3D(point0);
         expectedMovedPoint.addX(distance);

         assertEquals(1, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // start at beginning
         distance = 0.0;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(point0);
         expectedMovedPoint.addX(distance);

         assertEquals(0, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // test far in
         distance = 10.5;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(point0);
         expectedMovedPoint.setY(0.0);
         expectedMovedPoint.addX(distance);

         assertEquals(2, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // test at end
         distance = 10.8;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(point0);
         expectedMovedPoint.setY(0.0);
         expectedMovedPoint.addX(distance);

         assertEquals(2, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);

         // test past end
         distance = 14.0;
         segmentStart = BodyPathPlannerTools.movePointAlongBodyPath(bodyPath, startPoint, movedPoint, distance);

         expectedMovedPoint = new Point3D(point3);

         assertEquals(2, segmentStart);
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedMovedPoint, movedPoint, 1e-8);
      }
      // TODO Test if we start after the plan

   }
}

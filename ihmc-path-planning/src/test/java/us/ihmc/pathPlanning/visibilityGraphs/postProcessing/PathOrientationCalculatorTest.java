package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlannerTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.geometry.AngleTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class PathOrientationCalculatorTest
{
   @Test
   public void testStraightForward()
   {
      VisibilityGraphsParametersBasics visibilityGraphsParametersBasics = new DefaultVisibilityGraphParameters();
      PathOrientationCalculator pathOrientationCalculator = new PathOrientationCalculator(visibilityGraphsParametersBasics);


      List<Point3DReadOnly> points = new ArrayList<>();
      points.add(new Point3D());
      points.add(new Point3D(0.5, 0.0, 0.0));
      Orientation3DReadOnly zeroOrientation = new Quaternion();
      Orientation3DReadOnly startOrientation = zeroOrientation;
      Orientation3DReadOnly goalOrientation = zeroOrientation;

      List<? extends Pose3DReadOnly> poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(2, poses.size());
      for (int i = 0; i < poses.size(); i++)
         assertTrue(((Quaternion) zeroOrientation).distance(poses.get(i).getOrientation()) < 1e-5);

      points.add(new Point3D(1.0, 0.0, 0.0));

      poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(3, poses.size());
      for (int i = 0; i < poses.size(); i++)
         assertTrue(((Quaternion) zeroOrientation).distance(poses.get(i).getOrientation()) < 1e-5);


      points.clear();
      points.add(new Point3D());
      points.add(new Point3D(3.0, 0.0, 0.0));
      points.add(new Point3D(6.0, 0.0, 0.0));

      poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(3, poses.size());
      for (int i = 0; i < poses.size(); i++)
         assertTrue(((Quaternion) zeroOrientation).distance(poses.get(i).getOrientation()) < 1e-5);
   }

   @Test
   public void testStraightForwardWithHeightChange()
   {
      VisibilityGraphsParametersBasics visibilityGraphsParametersBasics = new DefaultVisibilityGraphParameters();
      PathOrientationCalculator pathOrientationCalculator = new PathOrientationCalculator(visibilityGraphsParametersBasics);


      List<Point3DReadOnly> points = new ArrayList<>();
      points.add(new Point3D());
      points.add(new Point3D(0.5, 0.0, 0.5));
      Orientation3DReadOnly zeroOrientation = new Quaternion();
      Orientation3DReadOnly startOrientation = zeroOrientation;
      Orientation3DReadOnly goalOrientation = zeroOrientation;

      List<? extends Pose3DReadOnly> poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(2, poses.size());
      for (int i = 0; i < poses.size(); i++)
         assertTrue(((Quaternion) zeroOrientation).distance(poses.get(i).getOrientation()) < 1e-5);

      points.add(new Point3D(1.0, 0.0, 0.5));

      poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(3, poses.size());
      for (int i = 0; i < poses.size(); i++)
         assertTrue(((Quaternion) zeroOrientation).distance(poses.get(i).getOrientation()) < 1e-5);


      points.clear();
      points.add(new Point3D());
      points.add(new Point3D(3.0, 0.0, 0.5));
      points.add(new Point3D(6.0, 0.0, 0.5));

      poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(3, poses.size());
      for (int i = 0; i < poses.size(); i++)
         assertTrue(((Quaternion) zeroOrientation).distance(poses.get(i).getOrientation()) < 1e-5);

      points.clear();
      points.add(new Point3D());
      points.add(new Point3D(2.9, 0.0, 0.0));
      points.add(new Point3D(3.0, 0.0, 0.5));
      points.add(new Point3D(6.0, 0.0, 0.5));

      poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(4, poses.size());
      for (int i = 0; i < poses.size(); i++)
         assertTrue(((Quaternion) zeroOrientation).distance(poses.get(i).getOrientation()) < 1e-5);
   }






   @Test
   public void testStepUpPlatform()
   {
      VisibilityGraphsParametersBasics visibilityGraphsParametersBasics = new DefaultVisibilityGraphParameters();
      PathOrientationCalculator pathOrientationCalculator = new PathOrientationCalculator(visibilityGraphsParametersBasics);


      List<Point3DReadOnly> points = new ArrayList<>();
      points.add(new Point3D(6.774, 0.046, -1.0));
      points.add(new Point3D(6.412, 0.046, -1.0));
      points.add(new Point3D(6.050, 0.046, -1.0));
      points.add(new Point3D(5.980, 0.046, -0.80));
      points.add(new Point3D(5.515, 0.046, -0.80));
      points.add(new Point3D(5.283, 0.046, -0.80));
      points.add(new Point3D(5.05, 0.046, -0.80));
      points.add(new Point3D(4.980, 0.046, -0.60));
      points.add(new Point3D(4.515, 0.046, -0.60));


      Quaternion zeroOrientation = new Quaternion();
      zeroOrientation.setYawPitchRoll(Math.PI, 0.0, 0.0);
      Orientation3DReadOnly startOrientation = zeroOrientation;
      Orientation3DReadOnly goalOrientation = zeroOrientation;

      List<? extends Pose3DReadOnly> poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(9, poses.size());
      assertTrue(poses.get(0).getOrientationDistance(zeroOrientation) < 1e-5);
      assertTrue(poses.get(poses.size() - 1).getOrientationDistance(zeroOrientation) < 1e-5);

      for (int i = 0; i < poses.size(); i++)
      {
         double actualYaw = poses.get(i).getOrientation().getYaw();
         double desiredYaw = zeroOrientation.getYaw();
         String errorMessage = "pose" + i + " had yaw " + actualYaw + ", but should have been " + desiredYaw;
         assertTrue(errorMessage, Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(actualYaw, desiredYaw)) < 1e-5);
      }

      points.clear();
      points.add(new Point3D(6.774, 0.046, -1.0));
      points.add(new Point3D(6.412, 0.106, -1.0));
      points.add(new Point3D(6.050, 0.167, -1.0));
      points.add(new Point3D(5.980, 0.096, -0.80));
      points.add(new Point3D(5.515, 0.131, -0.80));
      points.add(new Point3D(5.283, 0.149, -0.80));
      points.add(new Point3D(5.05, 0.167, -0.80));
      points.add(new Point3D(4.980, 0.096, -0.60));
      points.add(new Point3D(4.515, 0.131, -0.60));


      poses = pathOrientationCalculator.computePosesFromPath(points, null, startOrientation, goalOrientation);

      assertEquals(11, poses.size());
      assertTrue(poses.get(0).getOrientationDistance(zeroOrientation) < 1e-5);
      assertTrue(poses.get(poses.size() - 1).getOrientationDistance(zeroOrientation) < 1e-5);
      for (int i = 1; i < poses.size() - 1; i++)
      {
         double previousHeading = BodyPathPlannerTools.calculateHeading(poses.get(i - 1).getPosition(), poses.get(i).getPosition());
         if (i == 1)
            previousHeading = zeroOrientation.getYaw();

         double nextHeading = BodyPathPlannerTools.calculateHeading(poses.get(i).getPosition(), poses.get(i + 1).getPosition());
         double desiredOrientation = AngleTools.interpolateAngle(previousHeading, nextHeading, 0.5);
         double actualYaw = poses.get(i).getOrientation().getYaw();
         String errorMessage = "pose" + i + " had yaw " + actualYaw + ", but should have been " + desiredOrientation;
         assertTrue(errorMessage, Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(actualYaw, desiredOrientation)) < 1e-5);
      }

   }
}

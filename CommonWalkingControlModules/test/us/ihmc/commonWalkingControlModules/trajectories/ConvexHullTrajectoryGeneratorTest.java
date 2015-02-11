package us.ihmc.commonWalkingControlModules.trajectories;

import org.junit.Test;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.GroundOnlyQuadTree;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations;
import us.ihmc.utilities.math.dataStructures.DoubleHashHeightMap;
import us.ihmc.utilities.math.dataStructures.HeightMap;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import java.util.List;

import static org.junit.Assert.assertTrue;

/**
 * Created by agrabertilton on 2/11/15.
 */
public class ConvexHullTrajectoryGeneratorTest
{
   @Test
   public void testSmallXAxisDistanceWithoutHeightMap()
   {
      boolean VISUALIZE = false;
      double horizontalBuffer = .1; //10cm
      double verticalBuffer = 0.05; //5cm
      double pathWidth = 0.12; //12cm

      ConvexHullTrajectoryGenerator generator = new ConvexHullTrajectoryGenerator(horizontalBuffer, verticalBuffer, pathWidth);
      FramePose startPose = new FramePose(ReferenceFrame.getWorldFrame());
      FramePose endPose = new FramePose(ReferenceFrame.getWorldFrame());
      HeightMap groundMap = new DoubleHashHeightMap(0.01);

      Point3d startPosition = new Point3d(0.0, 0.0, 0.0);
      Quat4d startOrientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      startPose.setPose(startPosition, startOrientation);

      Point3d endPosition = new Point3d(startPosition.x + horizontalBuffer, startPosition.y, startPosition.z);
      Quat4d endOrientation = startOrientation;
      endPose.setPose(endPosition, endOrientation);

      List<FramePoint> trajectoryPoints = generator.computeSwingTrajectoryPoints(startPose, endPose, groundMap);
      assertTrue(trajectoryPoints.size() == 3);
      assertTrue(trajectoryPoints.get(0).epsilonEquals(startPosition, 1e-13));

      Point3d pointMiddle = new Point3d(startPosition);
      pointMiddle.add(endPosition);
      pointMiddle.scale(0.5);
      pointMiddle.add(new Point3d(0.0, 0.0, verticalBuffer));
      assertTrue(trajectoryPoints.get(1).epsilonEquals(pointMiddle, 1e-13));

      assertTrue(trajectoryPoints.get(2).epsilonEquals(endPosition, 1e-13));


      if (VISUALIZE)
      {
         visualizeTrajectoryPoints(trajectoryPoints);
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testBigXAxisDistanceWithoutHeightMap()
   {
      boolean VISUALIZE = false;
      double horizontalBuffer = .1; //10cm
      double verticalBuffer = 0.05; //5cm
      double pathWidth = 0.12; //12cm

      ConvexHullTrajectoryGenerator generator = new ConvexHullTrajectoryGenerator(horizontalBuffer, verticalBuffer, pathWidth);
      FramePose startPose = new FramePose(ReferenceFrame.getWorldFrame());
      FramePose endPose = new FramePose(ReferenceFrame.getWorldFrame());
      HeightMap groundMap = new DoubleHashHeightMap(0.01);

      Point3d startPosition = new Point3d(0.0, 0.0, 0.0);
      Quat4d startOrientation = new Quat4d(0.0, 0.0, 0.0, 1.0);
      startPose.setPose(startPosition, startOrientation);

      Point3d endPosition = new Point3d(startPosition.x + 3* horizontalBuffer, startPosition.y, startPosition.z);
      Quat4d endOrientation = startOrientation;
      endPose.setPose(endPosition, endOrientation);

      List<FramePoint> trajectoryPoints = generator.computeSwingTrajectoryPoints(startPose, endPose, groundMap);
      assertTrue(trajectoryPoints.size() == 4);
      assertTrue(trajectoryPoints.get(0).epsilonEquals(startPosition, 1e-13));

      Point3d pointA = new Point3d(startPosition.x + horizontalBuffer, startPosition.y, startPosition.z + verticalBuffer);
      assertTrue(trajectoryPoints.get(1).epsilonEquals(pointA, 1e-13));

      Point3d pointB = new Point3d(endPosition.x - horizontalBuffer, endPosition.y, endPosition.z + verticalBuffer);
      assertTrue(trajectoryPoints.get(2).epsilonEquals(pointB, 1e-13));

      assertTrue(trajectoryPoints.get(3).epsilonEquals(endPosition, 1e-13));


      if (VISUALIZE)
      {
         visualizeTrajectoryPoints(trajectoryPoints);
         ThreadTools.sleepForever();
      }
   }

   private void visualizeTrajectoryPoints(List<FramePoint> trajectoryPoints){
      Robot nullRobot = new Robot("FootstepVisualizerRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(nullRobot);
      scs.setDT(1, 1);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("SwingRegistry");
      BagOfBalls bagOfBalls = new BagOfBalls(registry, yoGraphicsListRegistry);
      for (FramePoint point : trajectoryPoints)
      {
         bagOfBalls.setBallLoop(point);
      }

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.addYoVariableRegistry(registry);
      scs.startOnAThread();
      scs.tickAndUpdate();
   }
}

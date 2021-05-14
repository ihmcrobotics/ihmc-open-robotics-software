package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;

public class SupportSequenceTest
{
   @Disabled
   @Test
   public void testSupportSequence()
   {
      ConvexPolygon2D defaultSupportPolygon = createDefaultSupportPolygon();
      SideDependentList<PoseReferenceFrame> soleFrames = createSoleFrames();
      SupportSequence supportSequence = new SupportSequence(defaultSupportPolygon, soleFrames, soleFrames);

      Footstep footstep = new Footstep(RobotSide.LEFT);
      footstep.setX(0.2);
      footstep.setY(0.1);
      footstep.setTrajectoryType(TrajectoryType.WAYPOINTS);
      FootstepTiming timing = new FootstepTiming(0.5, 0.2);
      timing.setLiftoffDuration(0.1);
      timing.setTouchdownDuration(0.1);

      RecyclingArrayList<FrameSE3TrajectoryPoint> swingTrajectory = new RecyclingArrayList<>(FrameSE3TrajectoryPoint.class);
      FrameQuaternion liftOffOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), 0.0, Math.toRadians(10.0), 0.0);
      FrameQuaternion touchDownOrientation = new FrameQuaternion(ReferenceFrame.getWorldFrame(), 0.0, Math.toRadians(-10.0), 0.0);
      swingTrajectory.add().set(new FrameSE3TrajectoryPoint(0.0, new FramePoint3D(), liftOffOrientation, new FrameVector3D(), new FrameVector3D()));
      swingTrajectory.add().set(new FrameSE3TrajectoryPoint(timing.getSwingTime(), new FramePoint3D(), touchDownOrientation, new FrameVector3D(), new FrameVector3D()));
      footstep.setSwingTrajectory(swingTrajectory);

      List<Footstep> footsteps = Arrays.asList(footstep);
      List<FootstepTiming> timings = Arrays.asList(timing);
      supportSequence.initializeStance();
      supportSequence.update(footsteps, timings);

      List<ConvexPolygon2D> expectedSupportPolygons = new ArrayList<>();
      TDoubleList expectedSupportTimes = new TDoubleArrayList();
      // At time 0.0 we expect full support
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.1, 0.15000000000000002);
         polygon.addVertex(0.1, 0.05);
         polygon.addVertex(0.1, -0.05);
         polygon.addVertex(0.1, -0.15000000000000002);
         polygon.addVertex(-0.1, -0.15000000000000002);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(-0.1, 0.05);
         polygon.addVertex(-0.1, 0.15000000000000002);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.0);
      }
      // At time 0.1 the robot should toe off with the left foot
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.1, 0.15);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(-0.1, -0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.1);
      }
      // At time 0.2 only the right foot should remain in contact
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.1, -0.05);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(-0.1, -0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.2);
      }
      // At time 0.7 the swing foot will have touched down with the heel
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.1, 0.15);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(-0.1, -0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.7);
      }
      // At time 0.8 the robot will be in full support
      {
         ConvexPolygon2D polygon = new ConvexPolygon2D();
         polygon.addVertex(0.3, 0.15);
         polygon.addVertex(0.3, 0.05);
         polygon.addVertex(0.1, -0.15);
         polygon.addVertex(-0.1, -0.15);
         polygon.addVertex(-0.1, -0.05);
         polygon.addVertex(0.1, 0.15);
         polygon.update();
         expectedSupportPolygons.add(polygon);
         expectedSupportTimes.add(0.8);
      }

      List<? extends ConvexPolygon2DReadOnly> supportPolygons = supportSequence.getSupportPolygons();
      TDoubleList supportTimes = supportSequence.getSupportTimes();
      for (int i = 0; i < supportPolygons.size(); i++)
      {
         EuclidGeometryTestTools.assertConvexPolygon2DGeometricallyEquals(expectedSupportPolygons.get(i), supportPolygons.get(i), 1.0e-1);
         Assert.assertEquals(expectedSupportTimes.get(i), supportTimes.get(i), 1.0e-10);
      }
   }

   private static SideDependentList<PoseReferenceFrame> createSoleFrames()
   {
      SideDependentList<PoseReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getLowerCaseName() + "SoleFrame", ReferenceFrame.getWorldFrame());
         soleFrame.setPositionWithoutChecksAndUpdate(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         soleFrames.put(robotSide, soleFrame);
      }
      return soleFrames;
   }

   private static ConvexPolygon2D createDefaultSupportPolygon()
   {
      ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
      defaultSupportPolygon.addVertex(0.1, 0.05);
      defaultSupportPolygon.addVertex(0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, 0.05);
      defaultSupportPolygon.addVertex(-0.1, -0.05);
      defaultSupportPolygon.update();
      return defaultSupportPolygon;
   }
}
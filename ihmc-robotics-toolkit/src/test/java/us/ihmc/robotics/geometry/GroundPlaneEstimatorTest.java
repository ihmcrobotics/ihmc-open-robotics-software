package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.MemoryTools;

public class GroundPlaneEstimatorTest
{
   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      ReferenceFrameTools.clearWorldFrameTree();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testPointsWithSamePitchAndDifferentPositionGetSameAnswer()
   {
      GroundPlaneEstimator groundPlaneEstimator = new GroundPlaneEstimator();
      List<FramePoint3D> pointListA = new ArrayList<FramePoint3D>();
      Plane3D plane3dA = new Plane3D();

      pointListA.add(new FramePoint3D(ReferenceFrame.getWorldFrame(),  1.0,  1.0,  0.1));
      pointListA.add(new FramePoint3D(ReferenceFrame.getWorldFrame(),  1.0, -1.0,  0.1));
      pointListA.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), -1.0,  1.0, -0.1));
      pointListA.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), -1.0, -1.0, -0.1));

      groundPlaneEstimator.compute(pointListA);
      groundPlaneEstimator.getPlane(plane3dA);
      Vector3D normalA = new Vector3D(plane3dA.getNormal());

      List<FramePoint3D> pointListB = new ArrayList<FramePoint3D>();
      Plane3D plane3dB = new Plane3D();
      pointListB.add(new FramePoint3D(ReferenceFrame.getWorldFrame(),  1.0 + 4.0,  1.0 + 4.0,  0.1));
      pointListB.add(new FramePoint3D(ReferenceFrame.getWorldFrame(),  1.0 + 4.0, -1.0 + 4.0,  0.1));
      pointListB.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), -1.0 + 4.0,  1.0 + 4.0, -0.1));
      pointListB.add(new FramePoint3D(ReferenceFrame.getWorldFrame(), -1.0 + 4.0, -1.0 + 4.0, -0.1));

      groundPlaneEstimator.compute(pointListB);
      groundPlaneEstimator.getPlane(plane3dB);
      Vector3D normalB = new Vector3D(plane3dB.getNormal());

      assertTrue(normalA.epsilonEquals(normalB, 1e-7));
   }

   @Test
   public void testPointsWithSamePitchAndDifferentPositionGetSameAnswer2()
   {
      double epsilon = 0.001;
      Random random = new Random(123123);

      GroundPlaneEstimator groundPlaneEstimator = new GroundPlaneEstimator();
      QuadrantDependentList<FramePoint3D> contactPoints = new QuadrantDependentList<>();
      FramePose3D centerOfFeetPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame centerOfFeetFrame = new PoseReferenceFrame("centerOfFeetFrame", centerOfFeetPose);

      //set feet random distance from each other on the same plane, max of 2 meters
      putFeetOnPlane(random, contactPoints, centerOfFeetFrame);

      //test random pitch between +-0.25 radians
      double actualPitch = 0.25;//random.nextDouble() * 0.5 - 0.25;
      centerOfFeetPose.getOrientation().setYawPitchRoll(centerOfFeetPose.getYaw(), actualPitch, centerOfFeetPose.getRoll());
      centerOfFeetFrame.setPoseAndUpdate(centerOfFeetPose);
      groundPlaneEstimator.compute(contactPoints);
      double computedPitch = groundPlaneEstimator.getPitch();
      assertEquals(actualPitch, computedPitch, epsilon);

      //change frame back before updating the feet frame
      changeFrame(contactPoints, centerOfFeetFrame);

      //move feet to random translation
      double x = random.nextInt(1000) + random.nextDouble();
      double y = random.nextInt(1000) + random.nextDouble();
      double z = random.nextInt(1000) + random.nextDouble();
      centerOfFeetPose.getPosition().set(x, y, z);
      centerOfFeetFrame.setPoseAndUpdate(centerOfFeetPose);
      groundPlaneEstimator.compute(contactPoints);
      computedPitch = groundPlaneEstimator.getPitch();

      assertEquals(actualPitch, computedPitch, epsilon);
   }

   @Disabled
   @Test
   public void testGetPitchWithFeetOnPlane()
   {
      double epsilon = 0.00001;
      Random random = new Random();

      GroundPlaneEstimator groundPlaneEstimator = new GroundPlaneEstimator();
      QuadrantDependentList<FramePoint3D> contactPoints = new QuadrantDependentList<>();
      FramePose3D centerOfFeetPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      PoseReferenceFrame centerOfFeetFrame = new PoseReferenceFrame("centerOfFeetFrame", centerOfFeetPose);

      //set feet random distance from each other on the same plane, max of 2 meters
      putFeetOnPlane(random, contactPoints, centerOfFeetFrame);

      //test zero pitch
      groundPlaneEstimator.compute(contactPoints);
      double actualPitch = centerOfFeetPose.getPitch();
      double computedPitch = groundPlaneEstimator.getPitch();
      assertTrue(MathTools.epsilonEquals(actualPitch, computedPitch, epsilon));

      //change frame back before updating the feet frame
      changeFrame(contactPoints, centerOfFeetFrame);

      //test pitch of 0.1 doesn't equal 0.0
      centerOfFeetPose.getOrientation().setYawPitchRoll(centerOfFeetPose.getYaw(), 0.1, centerOfFeetPose.getRoll());
      centerOfFeetFrame.setPoseAndUpdate(centerOfFeetPose);
      groundPlaneEstimator.compute(contactPoints);
      computedPitch = groundPlaneEstimator.getPitch();
      assertFalse(MathTools.epsilonEquals(0.0, computedPitch, epsilon));

      //change frame back before updating the feet frame
      changeFrame(contactPoints, centerOfFeetFrame);

      //test random pitch between +-0.25 radians
      actualPitch = random.nextDouble() * 0.5 - 0.25;
      centerOfFeetPose.getOrientation().setYawPitchRoll(centerOfFeetPose.getYaw(), actualPitch, centerOfFeetPose.getRoll());
      centerOfFeetFrame.setPoseAndUpdate(centerOfFeetPose);
      groundPlaneEstimator.compute(contactPoints);
      computedPitch = groundPlaneEstimator.getPitch();
      assertTrue(MathTools.epsilonEquals(actualPitch, computedPitch, epsilon));

      FrameVector3D normal = new FrameVector3D();
      FrameVector3D normal2 = new FrameVector3D();
      groundPlaneEstimator.getPlaneNormal(normal);

      //change frame back before updating the feet frame
      changeFrame(contactPoints, centerOfFeetFrame);

      //move feet to random translation
      double x = random.nextInt(1000) + random.nextDouble();
      double y = random.nextInt(1000) + random.nextDouble();
      double z = random.nextInt(1000) + random.nextDouble();
      centerOfFeetPose.getPosition().set(x, y, z);
      centerOfFeetFrame.setPoseAndUpdate(centerOfFeetPose);
      groundPlaneEstimator.compute(contactPoints);
      computedPitch = groundPlaneEstimator.getPitch();
      groundPlaneEstimator.getPlaneNormal(normal2);
      assertTrue(normal.epsilonEquals(normal2, epsilon));


      assertTrue(MathTools.epsilonEquals(actualPitch, computedPitch, epsilon));

      //test computing over several times doensn't change answer
      groundPlaneEstimator.compute(contactPoints);
      groundPlaneEstimator.compute(contactPoints);
      groundPlaneEstimator.compute(contactPoints);
      groundPlaneEstimator.compute(contactPoints);
      groundPlaneEstimator.compute(contactPoints);
      groundPlaneEstimator.compute(contactPoints);
      computedPitch = groundPlaneEstimator.getPitch();
      assertTrue(MathTools.epsilonEquals(actualPitch, computedPitch, epsilon));



   }

   private void putFeetOnPlane(Random random, QuadrantDependentList<FramePoint3D> contactPoints, PoseReferenceFrame centerOfFeetFrame)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D contactPoint = new FramePoint3D(centerOfFeetFrame);

         int xSign = (robotQuadrant.getEnd() == RobotEnd.FRONT) ? 1 : -1;
         contactPoint.setX(xSign * random.nextDouble());

         int ySign = robotQuadrant.getSide() == RobotSide.LEFT ? 1 : -1;
         contactPoint.setY(ySign * random.nextDouble());

         contactPoints.set(robotQuadrant, contactPoint);
      }
   }

   private void changeFrame(QuadrantDependentList<FramePoint3D> contactPoints, PoseReferenceFrame centerOfFeetFrame)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactPoints.get(robotQuadrant).changeFrame(centerOfFeetFrame);
      }
   }
}

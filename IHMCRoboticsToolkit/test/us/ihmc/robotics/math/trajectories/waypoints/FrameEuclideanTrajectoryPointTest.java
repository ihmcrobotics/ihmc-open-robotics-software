package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanTrajectoryPointTest
{

   @Test
   public void testConstructors()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = ReferenceFrame.generateRandomReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedPosition = new FramePoint(expectedFrame);
      expectedLinearVelocity = new FrameVector(expectedFrame);
      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFrameEuclideanTrajectoryPoint);

      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(testedFrameEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameEuclideanTrajectoryPoint.getReferenceFrame(), expectedFrameEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleEuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.setPosition(expectedFinalPosition.getPoint());
      expectedEuclideanTrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity.getVector());

      testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedFinalFrame, expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanTrajectoryPoint, epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = ReferenceFrame.generateRandomReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);

      final FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.set(expectedTime, expectedPosition.getPoint(), expectedLinearVelocity.getVector());

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameEuclideanTrajectoryPoint expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedFrameEuclideanTrajectoryPoint);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      expectedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.set(expectedFrameEuclideanTrajectoryPoint);

      assertTrue(expectedFrameEuclideanTrajectoryPoint.epsilonEquals(testedFrameEuclideanTrajectoryPoint, epsilon));
      assertTrajectoryPointContainsExpectedData(expectedFrameEuclideanTrajectoryPoint.getReferenceFrame(), expectedFrameEuclideanTrajectoryPoint.getTime(),
            expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SimpleEuclideanTrajectoryPoint expectedEuclideanTrajectoryPoint = new SimpleEuclideanTrajectoryPoint();
      expectedEuclideanTrajectoryPoint.setTime(expectedFinalTime);
      expectedEuclideanTrajectoryPoint.setPosition(expectedFinalPosition.getPoint());
      expectedEuclideanTrajectoryPoint.setLinearVelocity(expectedFinalLinearVelocity.getVector());

      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedFinalFrame, expectedEuclideanTrajectoryPoint);

      assertTrajectoryPointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanTrajectoryPoint, epsilon);

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         testedFrameEuclideanTrajectoryPoint.changeFrame(expectedFrame);

         assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedLinearVelocity.setToZero();
      testedFrameEuclideanTrajectoryPoint.setToZero();

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      testedFrameEuclideanTrajectoryPoint.setToZero(expectedFrame);

      assertTrajectoryPointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanTrajectoryPoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint = new FrameEuclideanTrajectoryPoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedFrameEuclideanTrajectoryPoint.containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameEuclideanTrajectoryPoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanTrajectoryPoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameEuclideanTrajectoryPoint.getTime()));
      assertTrue(testedFrameEuclideanTrajectoryPoint.containsNaN());
   }

   static void assertTrajectoryPointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameVector expectedLinearVelocity, FrameEuclideanTrajectoryPoint testedFrameEuclideanTrajectoryPoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameEuclideanTrajectoryPoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameEuclideanTrajectoryPoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedFrameEuclideanTrajectoryPoint.getGeometryObject().getPosition(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedFrameEuclideanTrajectoryPoint.getGeometryObject().getLinearVelocity(), epsilon));

      Point3d actualPosition = new Point3d();
      Vector3d actualLinearVelocity = new Vector3d();

      testedFrameEuclideanTrajectoryPoint.getPosition(actualPosition);
      testedFrameEuclideanTrajectoryPoint.getLinearVelocity(actualLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint();
      FrameVector actualFrameLinearVelocity = new FrameVector();

      testedFrameEuclideanTrajectoryPoint.getPositionIncludingFrame(actualFramePosition);
      testedFrameEuclideanTrajectoryPoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new FramePoint(expectedFrame);
      actualFrameLinearVelocity = new FrameVector(expectedFrame);

      testedFrameEuclideanTrajectoryPoint.getPosition(actualFramePosition);
      testedFrameEuclideanTrajectoryPoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }
}

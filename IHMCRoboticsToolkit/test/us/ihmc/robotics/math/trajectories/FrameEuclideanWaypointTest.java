package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEuclideanWaypointTest
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

      FrameEuclideanWaypoint testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedPosition = new FramePoint(expectedFrame);
      expectedLinearVelocity = new FrameVector(expectedFrame);
      testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedFrame);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedTime, expectedPosition, expectedLinearVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameEuclideanWaypoint expectedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedFrameEuclideanWaypoint);

      assertTrue(expectedFrameEuclideanWaypoint.epsilonEquals(testedFrameEuclideanWaypoint, epsilon));
      assertWaypointContainsExpectedData(expectedFrameEuclideanWaypoint.getReferenceFrame(), expectedFrameEuclideanWaypoint.getTime(),
            expectedFrameEuclideanWaypoint.getPosition(), expectedFrameEuclideanWaypoint.getLinearVelocity(), testedFrameEuclideanWaypoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      EuclideanWaypointInterface expectedDataHolder = new EuclideanWaypointInterface()
      {
         @Override
         public double getTime()
         {
            return expectedFinalTime;
         }

         @Override
         public void getPosition(Point3d positionToPack)
         {
            expectedFinalPosition.get(positionToPack);
         }

         @Override
         public void getLinearVelocity(Vector3d linearVelocityToPack)
         {
            expectedFinalLinearVelocity.get(linearVelocityToPack);
         }
      };

      testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedFinalFrame, expectedDataHolder);

      assertWaypointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanWaypoint, epsilon);

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

      final FrameEuclideanWaypoint testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanWaypoint.set(expectedTime, expectedPosition, expectedLinearVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanWaypoint.set(expectedTime, expectedPosition.getPoint(), expectedLinearVelocity.getVector());

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameEuclideanWaypoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameEuclideanWaypoint expectedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanWaypoint.setIncludingFrame(expectedFrameEuclideanWaypoint);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      expectedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanWaypoint.set(expectedFrameEuclideanWaypoint);

      assertTrue(expectedFrameEuclideanWaypoint.epsilonEquals(testedFrameEuclideanWaypoint, epsilon));
      assertWaypointContainsExpectedData(expectedFrameEuclideanWaypoint.getReferenceFrame(), expectedFrameEuclideanWaypoint.getTime(),
            expectedFrameEuclideanWaypoint.getPosition(), expectedFrameEuclideanWaypoint.getLinearVelocity(), testedFrameEuclideanWaypoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      EuclideanWaypointInterface expectedDataHolder = new EuclideanWaypointInterface()
      {
         @Override
         public double getTime()
         {
            return expectedFinalTime;
         }

         @Override
         public void getPosition(Point3d positionToPack)
         {
            expectedFinalPosition.get(positionToPack);
         }

         @Override
         public void getLinearVelocity(Vector3d linearVelocityToPack)
         {
            expectedFinalLinearVelocity.get(linearVelocityToPack);
         }
      };

      testedFrameEuclideanWaypoint.setIncludingFrame(expectedFinalFrame, expectedDataHolder);

      assertWaypointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalLinearVelocity,
            testedFrameEuclideanWaypoint, epsilon);

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
      FrameEuclideanWaypoint testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedTime, expectedPosition, expectedLinearVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         testedFrameEuclideanWaypoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);
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
      FrameEuclideanWaypoint testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedLinearVelocity.setToZero();
      testedFrameEuclideanWaypoint.setToZero();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameEuclideanWaypoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      testedFrameEuclideanWaypoint.setToZero(expectedFrame);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedLinearVelocity, testedFrameEuclideanWaypoint, epsilon);
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
      FrameEuclideanWaypoint testedFrameEuclideanWaypoint = new FrameEuclideanWaypoint(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanWaypoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameEuclideanWaypoint.getTime()));
      assertTrue(testedFrameEuclideanWaypoint.getPosition().containsNaN());
      assertTrue(testedFrameEuclideanWaypoint.getLinearVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameEuclideanWaypoint.setIncludingFrame(expectedTime, expectedPosition, expectedLinearVelocity);

      testedFrameEuclideanWaypoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameEuclideanWaypoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameEuclideanWaypoint.getTime()));
      assertTrue(testedFrameEuclideanWaypoint.getPosition().containsNaN());
      assertTrue(testedFrameEuclideanWaypoint.getLinearVelocity().containsNaN());
   }

   static void assertWaypointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameVector expectedLinearVelocity, FrameEuclideanWaypoint testedFrameEuclideanWaypoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameEuclideanWaypoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameEuclideanWaypoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedFrameEuclideanWaypoint.getPosition(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedFrameEuclideanWaypoint.getLinearVelocity(), epsilon));

      Point3d actualPosition = new Point3d();
      Vector3d actualLinearVelocity = new Vector3d();

      testedFrameEuclideanWaypoint.getPosition(actualPosition);
      testedFrameEuclideanWaypoint.getLinearVelocity(actualLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint();
      FrameVector actualFrameLinearVelocity = new FrameVector();

      testedFrameEuclideanWaypoint.getPositionIncludingFrame(actualFramePosition);
      testedFrameEuclideanWaypoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));

      actualFramePosition = new FramePoint(expectedFrame);
      actualFrameLinearVelocity = new FrameVector(expectedFrame);

      testedFrameEuclideanWaypoint.getPosition(actualFramePosition);
      testedFrameEuclideanWaypoint.getLinearVelocity(actualFrameLinearVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
   }
}

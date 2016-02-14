package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSO3WaypointTest
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
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      FrameSO3Waypoint testedFrameSO3Waypoint = new FrameSO3Waypoint();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedOrientation = new FrameOrientation(expectedFrame);
      expectedAngularVelocity = new FrameVector(expectedFrame);
      testedFrameSO3Waypoint = new FrameSO3Waypoint(expectedFrame);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3Waypoint = new FrameSO3Waypoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSO3Waypoint expectedFrameSO3Waypoint = new FrameSO3Waypoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3Waypoint = new FrameSO3Waypoint(expectedFrameSO3Waypoint);

      assertTrue(expectedFrameSO3Waypoint.epsilonEquals(testedFrameSO3Waypoint, epsilon));
      assertWaypointContainsExpectedData(expectedFrameSO3Waypoint.getReferenceFrame(), expectedFrameSO3Waypoint.getTime(),
            expectedFrameSO3Waypoint.getOrientation(), expectedFrameSO3Waypoint.getAngularVelocity(), testedFrameSO3Waypoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SO3WaypointInterface expectedDataHolder = new SO3WaypointInterface()
      {
         @Override
         public void getOrientation(Quat4d orientationToPack)
         {
            expectedFinalOrientation.getQuaternion(orientationToPack);
         }

         @Override
         public void getAngularVelocity(Vector3d angularVelocityToPack)
         {
            expectedFinalAngularVelocity.get(angularVelocityToPack);
         }

         @Override
         public double getTime()
         {
            return expectedFinalTime;
         }
      };

      testedFrameSO3Waypoint = new FrameSO3Waypoint(expectedFinalFrame, expectedDataHolder);

      assertWaypointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalOrientation, expectedFinalAngularVelocity, testedFrameSO3Waypoint,
            epsilon);

   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-15;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame aFrame = ReferenceFrame.generateRandomReferenceFrame("aFrame", random, worldFrame);

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      final FrameSO3Waypoint testedFrameSO3Waypoint = new FrameSO3Waypoint();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3Waypoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3Waypoint.set(expectedTime, expectedOrientation.getQuaternion(), expectedAngularVelocity.getVector());

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSO3Waypoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSO3Waypoint expectedFrameSO3Waypoint = new FrameSO3Waypoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3Waypoint.setIncludingFrame(expectedFrameSO3Waypoint);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      expectedFrameSO3Waypoint = new FrameSO3Waypoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3Waypoint.set(expectedFrameSO3Waypoint);

      assertTrue(expectedFrameSO3Waypoint.epsilonEquals(testedFrameSO3Waypoint, epsilon));
      assertWaypointContainsExpectedData(expectedFrameSO3Waypoint.getReferenceFrame(), expectedFrameSO3Waypoint.getTime(),
            expectedFrameSO3Waypoint.getOrientation(), expectedFrameSO3Waypoint.getAngularVelocity(), testedFrameSO3Waypoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SO3WaypointInterface expectedDataHolder = new SO3WaypointInterface()
      {
         @Override
         public void getOrientation(Quat4d orientationToPack)
         {
            expectedFinalOrientation.getQuaternion(orientationToPack);
         }

         @Override
         public void getAngularVelocity(Vector3d angularVelocityToPack)
         {
            expectedFinalAngularVelocity.get(angularVelocityToPack);
         }

         @Override
         public double getTime()
         {
            return expectedFinalTime;
         }
      };

      testedFrameSO3Waypoint.setIncludingFrame(expectedFinalFrame, expectedDataHolder);

      assertWaypointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalOrientation, expectedFinalAngularVelocity, testedFrameSO3Waypoint,
            epsilon);

   }

   @Test
   public void testChangeFrame() throws Exception
   {
      double epsilon = 1.0e-10;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSO3Waypoint testedFrameSO3Waypoint = new FrameSO3Waypoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedOrientation.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedFrameSO3Waypoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);
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
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSO3Waypoint testedFrameSO3Waypoint = new FrameSO3Waypoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero();
      expectedAngularVelocity.setToZero();
      testedFrameSO3Waypoint.setToZero();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSO3Waypoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedFrameSO3Waypoint.setToZero(expectedFrame);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedFrameSO3Waypoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSO3Waypoint testedFrameSO3Waypoint = new FrameSO3Waypoint(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3Waypoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameSO3Waypoint.getTime()));
      assertTrue(testedFrameSO3Waypoint.getOrientation().containsNaN());
      assertTrue(testedFrameSO3Waypoint.getAngularVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSO3Waypoint.setIncludingFrame(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedFrameSO3Waypoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameSO3Waypoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameSO3Waypoint.getTime()));
      assertTrue(testedFrameSO3Waypoint.getOrientation().containsNaN());
      assertTrue(testedFrameSO3Waypoint.getAngularVelocity().containsNaN());
   }

   static void assertWaypointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FrameOrientation expectedOrientation,
         FrameVector expectedAngularVelocity, FrameSO3Waypoint testedFrameSO3Waypoint, double epsilon)
   {
      assertTrue(expectedFrame == testedFrameSO3Waypoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameSO3Waypoint.getTime(), epsilon);
      assertTrue(expectedOrientation.epsilonEquals(testedFrameSO3Waypoint.getOrientation(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedFrameSO3Waypoint.getAngularVelocity(), epsilon));

      Quat4d actualOrientation = new Quat4d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedFrameSO3Waypoint.getOrientation(actualOrientation);
      testedFrameSO3Waypoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FrameOrientation actualFrameOrientation = new FrameOrientation();
      FrameVector actualFrameAngularVelocity = new FrameVector();

      testedFrameSO3Waypoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedFrameSO3Waypoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new FrameOrientation(expectedFrame);
      actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedFrameSO3Waypoint.getOrientation(actualFrameOrientation);
      testedFrameSO3Waypoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }
}

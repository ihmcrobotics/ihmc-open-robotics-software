package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.*;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameSE3WaypointTest
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
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      FrameSE3Waypoint testedFrameSE3Waypoint = new FrameSE3Waypoint();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = 0.0;
      expectedPosition = new FramePoint(expectedFrame);
      expectedOrientation = new FrameOrientation(expectedFrame);
      expectedLinearVelocity = new FrameVector(expectedFrame);
      expectedAngularVelocity = new FrameVector(expectedFrame);
      testedFrameSE3Waypoint = new FrameSE3Waypoint(expectedFrame);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3Waypoint = new FrameSE3Waypoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSE3Waypoint expectedFrameSE3Waypoint = new FrameSE3Waypoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      testedFrameSE3Waypoint = new FrameSE3Waypoint(expectedFrameSE3Waypoint);

      assertTrue(expectedFrameSE3Waypoint.epsilonEquals(testedFrameSE3Waypoint, epsilon));
      assertWaypointContainsExpectedData(expectedFrameSE3Waypoint.getReferenceFrame(), expectedFrameSE3Waypoint.getTime(),
            expectedFrameSE3Waypoint.getPosition(), expectedFrameSE3Waypoint.getOrientation(), expectedFrameSE3Waypoint.getLinearVelocity(),
            expectedFrameSE3Waypoint.getAngularVelocity(), testedFrameSE3Waypoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SE3WaypointInterface expectedDataHolder = new SE3WaypointInterface()
      {
         @Override public void getOrientation(Quat4d orientationToPack){expectedFinalOrientation.getQuaternion(orientationToPack);}
         @Override public void getAngularVelocity(Vector3d angularVelocityToPack){expectedFinalAngularVelocity.get(angularVelocityToPack);}
         @Override public double getTime(){return expectedFinalTime;}
         @Override public void getPosition(Point3d positionToPack){expectedFinalPosition.get(positionToPack);}
         @Override public void getLinearVelocity(Vector3d linearVelocityToPack){expectedFinalLinearVelocity.get(linearVelocityToPack);}
      };

      testedFrameSE3Waypoint = new FrameSE3Waypoint(expectedFinalFrame, expectedDataHolder);

      assertWaypointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalOrientation, expectedFinalLinearVelocity, expectedFinalAngularVelocity,
            testedFrameSE3Waypoint, epsilon);
      
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
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      final FrameSE3Waypoint testedFrameSE3Waypoint = new FrameSE3Waypoint();

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3Waypoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3Waypoint.set(expectedTime, expectedPosition.getPoint(), expectedOrientation.getQuaternion(), expectedLinearVelocity.getVector(), expectedAngularVelocity.getVector());

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = aFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedFrameSE3Waypoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      FrameSE3Waypoint expectedFrameSE3Waypoint = new FrameSE3Waypoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      testedFrameSE3Waypoint.setIncludingFrame(expectedFrameSE3Waypoint);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      expectedFrameSE3Waypoint = new FrameSE3Waypoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity,
            expectedAngularVelocity);

      testedFrameSE3Waypoint.set(expectedFrameSE3Waypoint);

      assertTrue(expectedFrameSE3Waypoint.epsilonEquals(testedFrameSE3Waypoint, epsilon));
      assertWaypointContainsExpectedData(expectedFrameSE3Waypoint.getReferenceFrame(), expectedFrameSE3Waypoint.getTime(),
            expectedFrameSE3Waypoint.getPosition(), expectedFrameSE3Waypoint.getOrientation(), expectedFrameSE3Waypoint.getLinearVelocity(),
            expectedFrameSE3Waypoint.getAngularVelocity(), testedFrameSE3Waypoint, epsilon);

      final ReferenceFrame expectedFinalFrame = aFrame;
      final double expectedFinalTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      final FramePoint expectedFinalPosition = FramePoint.generateRandomFramePoint(random, expectedFinalFrame, 10.0, 10.0, 10.0);
      final FrameOrientation expectedFinalOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFinalFrame);
      final FrameVector expectedFinalLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);
      final FrameVector expectedFinalAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFinalFrame);

      SE3WaypointInterface expectedDataHolder = new SE3WaypointInterface()
      {
         @Override public void getOrientation(Quat4d orientationToPack){expectedFinalOrientation.getQuaternion(orientationToPack);}
         @Override public void getAngularVelocity(Vector3d angularVelocityToPack){expectedFinalAngularVelocity.get(angularVelocityToPack);}
         @Override public double getTime(){return expectedFinalTime;}
         @Override public void getPosition(Point3d positionToPack){expectedFinalPosition.get(positionToPack);}
         @Override public void getLinearVelocity(Vector3d linearVelocityToPack){expectedFinalLinearVelocity.get(linearVelocityToPack);}
      };

      testedFrameSE3Waypoint.setIncludingFrame(expectedFinalFrame, expectedDataHolder);

      assertWaypointContainsExpectedData(expectedFinalFrame, expectedFinalTime, expectedFinalPosition, expectedFinalOrientation, expectedFinalLinearVelocity, expectedFinalAngularVelocity,
            testedFrameSE3Waypoint, epsilon);
      
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
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSE3Waypoint testedFrameSE3Waypoint = new FrameSE3Waypoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedOrientation.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedFrameSE3Waypoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
               testedFrameSE3Waypoint, epsilon);
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
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSE3Waypoint testedFrameSE3Waypoint = new FrameSE3Waypoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedOrientation.setToZero();
      expectedLinearVelocity.setToZero();
      expectedAngularVelocity.setToZero();
      testedFrameSE3Waypoint.setToZero();
      
      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSE3Waypoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedOrientation.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedFrameSE3Waypoint.setToZero(expectedFrame);
      
      assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedFrameSE3Waypoint, epsilon);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      FramePoint expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      FrameOrientation expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      FrameVector expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameVector expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      FrameSE3Waypoint testedFrameSE3Waypoint = new FrameSE3Waypoint(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3Waypoint.setToNaN();
      assertTrue(Double.isNaN(testedFrameSE3Waypoint.getTime()));
      assertTrue(testedFrameSE3Waypoint.getPosition().containsNaN());
      assertTrue(testedFrameSE3Waypoint.getOrientation().containsNaN());
      assertTrue(testedFrameSE3Waypoint.getLinearVelocity().containsNaN());
      assertTrue(testedFrameSE3Waypoint.getAngularVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedFrameSE3Waypoint.setIncludingFrame(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedFrameSE3Waypoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedFrameSE3Waypoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedFrameSE3Waypoint.getTime()));
      assertTrue(testedFrameSE3Waypoint.getPosition().containsNaN());
      assertTrue(testedFrameSE3Waypoint.getOrientation().containsNaN());
      assertTrue(testedFrameSE3Waypoint.getLinearVelocity().containsNaN());
      assertTrue(testedFrameSE3Waypoint.getAngularVelocity().containsNaN());
   }

   static void assertWaypointContainsExpectedData(ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameOrientation expectedOrientation, FrameVector expectedLinearVelocity, FrameVector expectedAngularVelocity, FrameSE3Waypoint testedFrameSE3Waypoint,
         double epsilon)
   {
      assertTrue(expectedFrame == testedFrameSE3Waypoint.getReferenceFrame());
      assertEquals(expectedTime, testedFrameSE3Waypoint.getTime(), epsilon);
      assertTrue(expectedPosition.epsilonEquals(testedFrameSE3Waypoint.getPosition(), epsilon));
      assertTrue(expectedOrientation.epsilonEquals(testedFrameSE3Waypoint.getOrientation(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedFrameSE3Waypoint.getLinearVelocity(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedFrameSE3Waypoint.getAngularVelocity(), epsilon));

      Point3d actualPosition = new Point3d();
      Quat4d actualOrientation = new Quat4d();
      Vector3d actualLinearVelocity = new Vector3d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedFrameSE3Waypoint.getPosition(actualPosition);
      testedFrameSE3Waypoint.getOrientation(actualOrientation);
      testedFrameSE3Waypoint.getLinearVelocity(actualLinearVelocity);
      testedFrameSE3Waypoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint();
      FrameOrientation actualFrameOrientation = new FrameOrientation();
      FrameVector actualFrameLinearVelocity = new FrameVector();
      FrameVector actualFrameAngularVelocity = new FrameVector();

      testedFrameSE3Waypoint.getPositionIncludingFrame(actualFramePosition);
      testedFrameSE3Waypoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedFrameSE3Waypoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);
      testedFrameSE3Waypoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new FramePoint(expectedFrame);
      actualFrameOrientation = new FrameOrientation(expectedFrame);
      actualFrameLinearVelocity = new FrameVector(expectedFrame);
      actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedFrameSE3Waypoint.getPosition(actualFramePosition);
      testedFrameSE3Waypoint.getOrientation(actualFrameOrientation);
      testedFrameSE3Waypoint.getLinearVelocity(actualFrameLinearVelocity);
      testedFrameSE3Waypoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }
}

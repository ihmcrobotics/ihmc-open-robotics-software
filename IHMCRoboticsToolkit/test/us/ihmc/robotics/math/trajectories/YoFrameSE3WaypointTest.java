package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSE3WaypointTest
{

   @Test
   public void testConstructor()
   {
      double epsilon = 1.0e-20;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3Waypoint testedYoFrameSE3Waypoint = new YoFrameSE3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedYoFrameSE3Waypoint, epsilon);
   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FramePoint expectedPosition = new FramePoint(expectedFrame);
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedLinearVelocity = new FrameVector(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3Waypoint testedYoFrameSE3Waypoint = new YoFrameSE3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedYoFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSE3Waypoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedYoFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSE3Waypoint.set(expectedTime, expectedPosition.getPoint(), expectedOrientation.getQuaternion(), expectedLinearVelocity.getVector(), expectedAngularVelocity.getVector());

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedYoFrameSE3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, expectedFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      YoFrameSE3Waypoint expectedYoFrameSE3Waypoint = new YoFrameSE3Waypoint("sdfsd", "asd", new YoVariableRegistry("asawe"), expectedFrame);

      testedYoFrameSE3Waypoint.set(expectedYoFrameSE3Waypoint);
      FrameSE3Waypoint expectedFrameSE3Waypoint = expectedYoFrameSE3Waypoint.getFrameSE3WaypointCopy();

      assertTrue(expectedYoFrameSE3Waypoint.epsilonEquals(testedYoFrameSE3Waypoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrameSE3Waypoint.getReferenceFrame(), expectedFrameSE3Waypoint.getTime(),
            expectedFrameSE3Waypoint.getPosition(), expectedFrameSE3Waypoint.getOrientation(), expectedFrameSE3Waypoint.getLinearVelocity(),
            expectedFrameSE3Waypoint.getAngularVelocity(), testedYoFrameSE3Waypoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3Waypoint testedYoFrameSE3Waypoint = new YoFrameSE3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      
      testedYoFrameSE3Waypoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);
      
      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameSE3Waypoint.registerReferenceFrame(expectedFrame);

         expectedPosition.changeFrame(expectedFrame);
         expectedOrientation.changeFrame(expectedFrame);
         expectedLinearVelocity.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedYoFrameSE3Waypoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
               testedYoFrameSE3Waypoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3Waypoint testedYoFrameSE3Waypoint = new YoFrameSE3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSE3Waypoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero();
      expectedOrientation.setToZero();
      expectedLinearVelocity.setToZero();
      expectedAngularVelocity.setToZero();
      testedYoFrameSE3Waypoint.setToZero();
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedYoFrameSE3Waypoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSE3Waypoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSE3Waypoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSE3Waypoint.registerReferenceFrame(worldFrame);
      testedYoFrameSE3Waypoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedPosition.setToZero(expectedFrame);
      expectedOrientation.setToZero(expectedFrame);
      expectedLinearVelocity.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedYoFrameSE3Waypoint.switchCurrentReferenceFrame(expectedFrame);
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity,
            testedYoFrameSE3Waypoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSE3Waypoint testedYoFrameSE3Waypoint = new YoFrameSE3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSE3Waypoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedYoFrameSE3Waypoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameSE3Waypoint.getTime()));
      assertTrue(testedYoFrameSE3Waypoint.getPosition().containsNaN());
      assertTrue(testedYoFrameSE3Waypoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSE3Waypoint.getLinearVelocity().containsNaN());
      assertTrue(testedYoFrameSE3Waypoint.getAngularVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSE3Waypoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 10.0, 10.0, 10.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedLinearVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSE3Waypoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSE3Waypoint.registerReferenceFrame(worldFrame);
      testedYoFrameSE3Waypoint.set(expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity);

      testedYoFrameSE3Waypoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameSE3Waypoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameSE3Waypoint.getTime()));
      assertTrue(testedYoFrameSE3Waypoint.getPosition().containsNaN());
      assertTrue(testedYoFrameSE3Waypoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSE3Waypoint.getLinearVelocity().containsNaN());
      assertTrue(testedYoFrameSE3Waypoint.getAngularVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime, FramePoint expectedPosition,
         FrameOrientation expectedOrientation, FrameVector expectedLinearVelocity, FrameVector expectedAngularVelocity, YoFrameSE3Waypoint testedYoFrameSE3Waypoint,
         double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameSE3Waypoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameSE3Waypoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameSE3Waypoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameSE3Waypoint.getNameSuffix());
      assertTrue(expectedPosition.epsilonEquals(testedYoFrameSE3Waypoint.getPosition().getFramePointCopy(), epsilon));
      assertTrue(expectedOrientation.epsilonEquals(testedYoFrameSE3Waypoint.getOrientation().getFrameOrientationCopy(), epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(testedYoFrameSE3Waypoint.getLinearVelocity().getFrameVectorCopy(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedYoFrameSE3Waypoint.getAngularVelocity().getFrameVectorCopy(), epsilon));

      FrameSE3Waypoint actualFrameSE3Waypoint = new FrameSE3Waypoint();
      testedYoFrameSE3Waypoint.getFrameSE3WaypointIncludingFrame(actualFrameSE3Waypoint);
      FrameSE3WaypointTest.assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, actualFrameSE3Waypoint, epsilon);
      actualFrameSE3Waypoint = new FrameSE3Waypoint(expectedFrame);
      testedYoFrameSE3Waypoint.getFrameSE3Waypoint(actualFrameSE3Waypoint);
      FrameSE3WaypointTest.assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedPosition, expectedOrientation, expectedLinearVelocity, expectedAngularVelocity, actualFrameSE3Waypoint, epsilon);

      Point3d actualPosition = new Point3d();
      Quat4d actualOrientation = new Quat4d();
      Vector3d actualLinearVelocity = new Vector3d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedYoFrameSE3Waypoint.getPosition(actualPosition);
      testedYoFrameSE3Waypoint.getOrientation(actualOrientation);
      testedYoFrameSE3Waypoint.getLinearVelocity(actualLinearVelocity);
      testedYoFrameSE3Waypoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualPosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FramePoint actualFramePosition = new FramePoint(expectedFrame);
      FrameOrientation actualFrameOrientation = new FrameOrientation(expectedFrame);
      FrameVector actualFrameLinearVelocity = new FrameVector(expectedFrame);
      FrameVector actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedYoFrameSE3Waypoint.getPosition(actualFramePosition);
      testedYoFrameSE3Waypoint.getOrientation(actualFrameOrientation);
      testedYoFrameSE3Waypoint.getLinearVelocity(actualFrameLinearVelocity);
      testedYoFrameSE3Waypoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFramePosition = new FramePoint();
      actualFrameOrientation = new FrameOrientation();
      actualFrameLinearVelocity = new FrameVector();
      actualFrameAngularVelocity = new FrameVector();

      testedYoFrameSE3Waypoint.getPositionIncludingFrame(actualFramePosition);
      testedYoFrameSE3Waypoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedYoFrameSE3Waypoint.getLinearVelocityIncludingFrame(actualFrameLinearVelocity);
      testedYoFrameSE3Waypoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedPosition.epsilonEquals(actualFramePosition, epsilon));
      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedLinearVelocity.epsilonEquals(actualFrameLinearVelocity, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }

}

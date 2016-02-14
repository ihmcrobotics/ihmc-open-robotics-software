package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoFrameSO3WaypointTest
{

   @Test
   public void testConstructor()
   {
      double epsilon = 1.0e-20;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3Waypoint testedYoFrameSO3Waypoint = new YoFrameSO3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedYoFrameSO3Waypoint, epsilon);
   }

   @Test
   public void testSetters()
   {
      double epsilon = 1.0e-20;
      Random random = new Random(21651016L);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      ReferenceFrame expectedFrame = worldFrame;
      double expectedTime = 0.0;
      FrameOrientation expectedOrientation = new FrameOrientation(expectedFrame);
      FrameVector expectedAngularVelocity = new FrameVector(expectedFrame);

      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3Waypoint testedYoFrameSO3Waypoint = new YoFrameSO3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedYoFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSO3Waypoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedYoFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      testedYoFrameSO3Waypoint.set(expectedTime, expectedOrientation.getQuaternion(), expectedAngularVelocity.getVector());

      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedYoFrameSO3Waypoint, epsilon);

      expectedFrame = worldFrame;
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, expectedFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, expectedFrame);

      YoFrameSO3Waypoint expectedYoFrameSO3Waypoint = new YoFrameSO3Waypoint("sdfsd", "asd", new YoVariableRegistry("asawe"), expectedFrame);

      testedYoFrameSO3Waypoint.set(expectedYoFrameSO3Waypoint);
      FrameSO3Waypoint expectedFrameSO3Waypoint = expectedYoFrameSO3Waypoint.getFrameSO3WaypointCopy();

      assertTrue(expectedYoFrameSO3Waypoint.epsilonEquals(testedYoFrameSO3Waypoint, epsilon));
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrameSO3Waypoint.getReferenceFrame(), expectedFrameSO3Waypoint.getTime(),
            expectedFrameSO3Waypoint.getOrientation(), expectedFrameSO3Waypoint.getAngularVelocity(), testedYoFrameSO3Waypoint,
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3Waypoint testedYoFrameSO3Waypoint = new YoFrameSO3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      
      testedYoFrameSO3Waypoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      ReferenceFrame[] randomFrames = new ReferenceFrame[10];
      randomFrames[0] = worldFrame;
      for (int i = 1; i < 10; i++)
         randomFrames[i] = ReferenceFrame.generateRandomReferenceFrame("randomFrame" + i, random, random.nextBoolean() ? worldFrame : randomFrames[random.nextInt(i)]);
      
      for (int i = 0; i < 10000; i++)
      {
         expectedFrame = randomFrames[random.nextInt(10)];
         testedYoFrameSO3Waypoint.registerReferenceFrame(expectedFrame);

         expectedOrientation.changeFrame(expectedFrame);
         expectedAngularVelocity.changeFrame(expectedFrame);
         testedYoFrameSO3Waypoint.changeFrame(expectedFrame);

         assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedYoFrameSO3Waypoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3Waypoint testedYoFrameSO3Waypoint = new YoFrameSO3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSO3Waypoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero();
      expectedAngularVelocity.setToZero();
      testedYoFrameSO3Waypoint.setToZero();
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedYoFrameSO3Waypoint, epsilon);

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3Waypoint.registerReferenceFrame(expectedFrame);

      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSO3Waypoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSO3Waypoint.registerReferenceFrame(worldFrame);
      testedYoFrameSO3Waypoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      expectedTime = 0.0;
      expectedOrientation.setToZero(expectedFrame);
      expectedAngularVelocity.setToZero(expectedFrame);
      testedYoFrameSO3Waypoint.switchCurrentReferenceFrame(expectedFrame);
      
      assertWaypointContainsExpectedData(expectedNamePrefix, expectedNameSuffix, expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, testedYoFrameSO3Waypoint, epsilon);
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
      String expectedNamePrefix = "test";
      String expectedNameSuffix = "blop";
      YoFrameSO3Waypoint testedYoFrameSO3Waypoint = new YoFrameSO3Waypoint(expectedNamePrefix, expectedNameSuffix, new YoVariableRegistry("schnoop"), expectedFrame);
      testedYoFrameSO3Waypoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedYoFrameSO3Waypoint.setToNaN();
      assertTrue(Double.isNaN(testedYoFrameSO3Waypoint.getTime()));
      assertTrue(testedYoFrameSO3Waypoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSO3Waypoint.getAngularVelocity().containsNaN());

      expectedFrame = ReferenceFrame.generateRandomReferenceFrame("blop", random, worldFrame);
      testedYoFrameSO3Waypoint.registerReferenceFrame(expectedFrame);
      expectedTime = RandomTools.generateRandomDouble(random, 0.0, 1000.0);
      expectedOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      expectedAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
      testedYoFrameSO3Waypoint.switchCurrentReferenceFrame(worldFrame);
      testedYoFrameSO3Waypoint.registerReferenceFrame(worldFrame);
      testedYoFrameSO3Waypoint.set(expectedTime, expectedOrientation, expectedAngularVelocity);

      testedYoFrameSO3Waypoint.setToNaN(expectedFrame);

      assertTrue(expectedFrame == testedYoFrameSO3Waypoint.getReferenceFrame());
      assertTrue(Double.isNaN(testedYoFrameSO3Waypoint.getTime()));
      assertTrue(testedYoFrameSO3Waypoint.getOrientation().containsNaN());
      assertTrue(testedYoFrameSO3Waypoint.getAngularVelocity().containsNaN());
   }

   private void assertWaypointContainsExpectedData(String expectedNamePrefix, String expectedNameSuffix, ReferenceFrame expectedFrame, double expectedTime, FrameOrientation expectedOrientation,
         FrameVector expectedAngularVelocity, YoFrameSO3Waypoint testedYoFrameSO3Waypoint, double epsilon)
   {
      assertTrue(expectedFrame == testedYoFrameSO3Waypoint.getReferenceFrame());
      assertEquals(expectedTime, testedYoFrameSO3Waypoint.getTime(), epsilon);
      assertEquals(expectedNamePrefix, testedYoFrameSO3Waypoint.getNamePrefix());
      assertEquals(expectedNameSuffix, testedYoFrameSO3Waypoint.getNameSuffix());
      assertTrue(expectedOrientation.epsilonEquals(testedYoFrameSO3Waypoint.getOrientation().getFrameOrientationCopy(), epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(testedYoFrameSO3Waypoint.getAngularVelocity().getFrameVectorCopy(), epsilon));

      FrameSO3Waypoint actualFrameSO3Waypoint = new FrameSO3Waypoint();
      testedYoFrameSO3Waypoint.getFrameSO3WaypointIncludingFrame(actualFrameSO3Waypoint);
      FrameSO3WaypointTest.assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, actualFrameSO3Waypoint, epsilon);
      actualFrameSO3Waypoint = new FrameSO3Waypoint(expectedFrame);
      testedYoFrameSO3Waypoint.getFrameSO3Waypoint(actualFrameSO3Waypoint);
      FrameSO3WaypointTest.assertWaypointContainsExpectedData(expectedFrame, expectedTime, expectedOrientation, expectedAngularVelocity, actualFrameSO3Waypoint, epsilon);

      Quat4d actualOrientation = new Quat4d();
      Vector3d actualAngularVelocity = new Vector3d();

      testedYoFrameSO3Waypoint.getOrientation(actualOrientation);
      testedYoFrameSO3Waypoint.getAngularVelocity(actualAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualAngularVelocity, epsilon));

      FrameOrientation actualFrameOrientation = new FrameOrientation(expectedFrame);
      FrameVector actualFrameAngularVelocity = new FrameVector(expectedFrame);

      testedYoFrameSO3Waypoint.getOrientation(actualFrameOrientation);
      testedYoFrameSO3Waypoint.getAngularVelocity(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));

      actualFrameOrientation = new FrameOrientation();
      actualFrameAngularVelocity = new FrameVector();

      testedYoFrameSO3Waypoint.getOrientationIncludingFrame(actualFrameOrientation);
      testedYoFrameSO3Waypoint.getAngularVelocityIncludingFrame(actualFrameAngularVelocity);

      assertTrue(expectedOrientation.epsilonEquals(actualFrameOrientation, epsilon));
      assertTrue(expectedAngularVelocity.epsilonEquals(actualFrameAngularVelocity, epsilon));
   }
}

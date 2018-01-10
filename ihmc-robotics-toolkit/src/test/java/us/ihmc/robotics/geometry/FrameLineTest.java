package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.OrientationFrame;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class FrameLineTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testChangeFrameCopy()
   {
      Random random = new Random(1776L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameLine3D lWorld = new FrameLine3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.0, -2.0, -3.0), new Vector3D(1.0, 2.0, 3.0));

      ArrayList<ReferenceFrame> frames = new ArrayList<ReferenceFrame>();
      frames.add(worldFrame);

      int numberOfFrames = 10;
      ReferenceFrame parentFrame = worldFrame;

      for (int i = 0; i < numberOfFrames; i++)
      {
         ReferenceFrame frame = createRandomFrame(parentFrame, random);
         frames.add(frame);
         parentFrame = frame;
      }

      ArrayList<FrameLine3D> resultLines = new ArrayList<FrameLine3D>();
      resultLines.add(lWorld);

      // Choose random paths and move the vectors around those paths:
      int numVectors = 1000;

      for (int i = 0; i < numVectors; i++)
      {
         int pathLength = random.nextInt(20);

         FrameLine3D line = lWorld;

         for (int j = 0; j < pathLength; j++)
         {
            int frameIndex = random.nextInt(frames.size());
            line = new FrameLine3D(line);
            line.changeFrame(frames.get(frameIndex));
         }

         resultLines.add(line);
      }

      // Now compare all sets of 2 vectors. If they are in the same frame, they should have the same values
      for (FrameLine3D resultLine1 : resultLines)
      {
         // Print out the vectors:

         for (FrameLine3D resultLine2 : resultLines)
         {
            if (resultLine1.getReferenceFrame() == resultLine2.getReferenceFrame())
            {
               checkEquals(resultLine1, resultLine2);
            }
         }
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFrameVector()
   {
      Random random = new Random(1234L);

      ReferenceFrame world = ReferenceFrame.getWorldFrame();
      ReferenceFrame target = createRandomFrame(world, random);
      ReferenceFrame target2 = createRandomFrame(world, random);

      Point3D origin = new Point3D();
      Vector3D direction = new Vector3D(1.0, 2.0, 3.0);

      FrameLine3D line = new FrameLine3D(world, origin, direction);
      FrameVector3D vector = new FrameVector3D(world, direction);
      vector.normalize();

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      line.changeFrame(target);
      vector.changeFrame(target);

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      vector.changeFrame(target2);
      FrameVector3D frameVector = line.getFrameNormalizedVectorCopy();
      frameVector.changeFrame(target2);
      assertTrue(frameVector.epsilonEquals(vector, 1e-12));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetFramePoint()
   {
      Random random = new Random(4567L);

      ReferenceFrame world = ReferenceFrame.getWorldFrame();
      ReferenceFrame target = createRandomFrame(world, random);
      ReferenceFrame target2 = createRandomFrame(world, random);

      Point3D origin = new Point3D(1.0, 2.0, 3.0);
      Vector3D direction = new Vector3D(2.0, 1.0, 4.0);

      FrameLine3D line = new FrameLine3D(world, origin, direction);
      FrameVector3D vector = new FrameVector3D(world, direction);
      vector.normalize();

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      line.changeFrame(target);
      vector.changeFrame(target);

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      vector.changeFrame(target2);
      FrameVector3D frameNormalizedVectorCopy = line.getFrameNormalizedVectorCopy();
      frameNormalizedVectorCopy.changeFrame(target2);
      assertTrue(frameNormalizedVectorCopy.epsilonEquals(vector, 1e-12));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testConstructorC()
   {
      FramePoint3D origin = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameVector3D direction = new FrameVector3D(createRandomFrame(ReferenceFrame.getWorldFrame(), new Random(1231L)), 4.0, 5.0, 6.0);

      new FrameLine3D(origin, direction);
   }

   private ReferenceFrame createRandomFrame(ReferenceFrame parentFrame, Random random)
   {
      FrameQuaternion orientation = new FrameQuaternion(parentFrame, createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0),
            createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0), createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0));

      OrientationFrame frame = new OrientationFrame(orientation);

      return frame;
   }

   private double createRandomDouble(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   private void checkEquals(FrameLine3D v1, FrameLine3D v2)
   {
      if (!v1.epsilonEquals(v2, 1e-7))
      {
         throw new RuntimeException("Frame Vectors don't match. v1 = " + v1 + ", v2 = " + v2);
      }

   }

}

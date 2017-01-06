package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class FrameLineTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testChangeFrameCopy()
   {
      Random random = new Random(1776L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameLine lWorld = new FrameLine(ReferenceFrame.getWorldFrame(), new Point3d(-1.0, -2.0, -3.0), new Vector3d(1.0, 2.0, 3.0));

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

      ArrayList<FrameLine> resultLines = new ArrayList<FrameLine>();
      resultLines.add(lWorld);

      // Choose random paths and move the vectors around those paths:
      int numVectors = 1000;

      for (int i = 0; i < numVectors; i++)
      {
         int pathLength = random.nextInt(20);

         FrameLine line = lWorld;

         for (int j = 0; j < pathLength; j++)
         {
            int frameIndex = random.nextInt(frames.size());
            line = new FrameLine(line);
            line.changeFrame(frames.get(frameIndex));
         }

         resultLines.add(line);
      }

      // Now compare all sets of 2 vectors. If they are in the same frame, they should have the same values
      for (FrameLine resultLine1 : resultLines)
      {
         // Print out the vectors:

         for (FrameLine resultLine2 : resultLines)
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

      Point3d origin = new Point3d();
      Vector3d direction = new Vector3d(1.0, 2.0, 3.0);

      FrameLine line = new FrameLine(world, origin, direction);
      FrameVector vector = new FrameVector(world, direction);
      vector.normalize();

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      line.changeFrame(target);
      vector.changeFrame(target);

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      vector.changeFrame(target2);
      FrameVector frameVector = line.getFrameNormalizedVectorCopy();
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

      Point3d origin = new Point3d(1.0, 2.0, 3.0);
      Vector3d direction = new Vector3d(2.0, 1.0, 4.0);

      FrameLine line = new FrameLine(world, origin, direction);
      FrameVector vector = new FrameVector(world, direction);
      vector.normalize();

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      line.changeFrame(target);
      vector.changeFrame(target);

      assertTrue(vector.epsilonEquals(line.getFrameNormalizedVectorCopy(), 1e-12));

      vector.changeFrame(target2);
      FrameVector frameNormalizedVectorCopy = line.getFrameNormalizedVectorCopy();
      frameNormalizedVectorCopy.changeFrame(target2);
      assertTrue(frameNormalizedVectorCopy.epsilonEquals(vector, 1e-12));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructorA()
   {
      Point3d origin = new Point3d(1.0, 2.0, 3.0);
      Vector3d direction = new Vector3d();

      FrameLine frameLine = new FrameLine(ReferenceFrame.getWorldFrame(), origin, direction);
      
      assertTrue(frameLine.containsNaN());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructorB()
   {
      FramePoint origin = new FramePoint(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameVector direction = new FrameVector(ReferenceFrame.getWorldFrame());

      FrameLine frameLine = new FrameLine(origin, direction);
      
      assertTrue(frameLine.containsNaN());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testConstructorC()
   {
      FramePoint origin = new FramePoint(ReferenceFrame.getWorldFrame(), 1.0, 2.0, 3.0);
      FrameVector direction = new FrameVector(createRandomFrame(ReferenceFrame.getWorldFrame(), new Random(1231L)), 4.0, 5.0, 6.0);

      new FrameLine(origin, direction);
   }

   private ReferenceFrame createRandomFrame(ReferenceFrame parentFrame, Random random)
   {
      FrameOrientation orientation = new FrameOrientation(parentFrame, createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0),
            createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0), createRandomDouble(random, -Math.PI / 3.0, Math.PI / 3.0));

      OrientationFrame frame = new OrientationFrame(orientation);

      return frame;
   }

   private double createRandomDouble(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   private void checkEquals(FrameLine v1, FrameLine v2)
   {
      if (!v1.epsilonEquals(v2, 1e-7))
      {
         throw new RuntimeException("Frame Vectors don't match. v1 = " + v1 + ", v2 = " + v2);
      }

   }

}

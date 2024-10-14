package us.ihmc.robotics.kinematics;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 4/25/13
 * Time: 3:50 PM
 * To change this template use File | Settings | File Templates.
 */
public class TransformInterpolationCalculatorTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testInterpolationForTimeStampedTransform()
   {
      Random random = new Random(52156165L);
      TimeStampedTransform3D firstTimeStampedTransform = new TimeStampedTransform3D();
      TimeStampedTransform3D secondTimeStampedTransform = new TimeStampedTransform3D();
      TimeStampedTransform3D toTestTimeStampedTransform = new TimeStampedTransform3D();
      
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      
      firstTimeStampedTransform.setTimeStamp(RandomNumbers.nextInt(random, 123, 45196516));
      secondTimeStampedTransform.setTimeStamp(firstTimeStampedTransform.getTimeStamp() - 1);
      
      try
      {
         TransformInterpolationCalculator.interpolate(firstTimeStampedTransform, secondTimeStampedTransform, toTestTimeStampedTransform, 216515L);
         fail("Should have thrown a RuntimeException as firstTimestamp is smaller than secondTimestamp.");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      firstTimeStampedTransform.setTimeStamp(RandomNumbers.nextInt(random, 123, 45196516));
      secondTimeStampedTransform.setTimeStamp(firstTimeStampedTransform.getTimeStamp() + RandomNumbers.nextInt(random, 1, 20));

      try
      {
         TransformInterpolationCalculator.interpolate(firstTimeStampedTransform, secondTimeStampedTransform, toTestTimeStampedTransform, secondTimeStampedTransform.getTimeStamp() + 1);
         fail("Should have thrown a RuntimeException as the given timestamp is outside bounds.");
      }
      catch (RuntimeException e)
      {
         // Good
      }


      try
      {
         TransformInterpolationCalculator.interpolate(firstTimeStampedTransform, secondTimeStampedTransform, toTestTimeStampedTransform, firstTimeStampedTransform.getTimeStamp() - 1);
         fail("Should have thrown a RuntimeException as the given timestamp is outside bounds.");
      }
      catch (RuntimeException e)
      {
         // Good
      }

      for (int i = 0; i < 100; i++)
      {
         firstTimeStampedTransform.setTransform3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));
         secondTimeStampedTransform.setTransform3D(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         long timestamp1 = RandomNumbers.nextInt(random, 123, Integer.MAX_VALUE / 4);
         long timestamp2 = timestamp1 + RandomNumbers.nextInt(random, 1, 200);
         firstTimeStampedTransform.setTimeStamp(timestamp1);
         secondTimeStampedTransform.setTimeStamp(timestamp2);

         long timeStampForInterpolation = RandomNumbers.nextInt(random, (int) firstTimeStampedTransform.getTimeStamp(), (int) secondTimeStampedTransform.getTimeStamp());

         TransformInterpolationCalculator.interpolate(firstTimeStampedTransform, secondTimeStampedTransform, toTestTimeStampedTransform, timeStampForInterpolation);

         double alpha = ((double) timeStampForInterpolation - (double) timestamp1) / ((double) timestamp2 - (double) timestamp1);
         expectedTransform.interpolate(firstTimeStampedTransform.getTransform3D(), secondTimeStampedTransform.getTransform3D(), alpha);

         assertTrue(expectedTransform.epsilonEquals(toTestTimeStampedTransform.getTransform3D(), 1.0e-10));
      }
   }
}

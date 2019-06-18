package us.ihmc.quadrupedRobotics.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.GroundPlaneEstimator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PitchEstimatorTest
{
   private static final int iters = 1000;

   @Test
   public void testCalculatedPitch()
   {
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double nominalLength = RandomNumbers.nextDouble(random, 0.5, 1.5);
         double nominalWidth = RandomNumbers.nextDouble(random, 0.15, 0.8 * nominalLength);


         Point3D translation = EuclidCoreRandomTools.nextPoint3D(random, 20.0);
         Orientation3DBasics rotation = new AxisAngle(RandomNumbers.nextDouble(random, -Math.PI, Math.PI), 0.0, 0.0);
         PoseReferenceFrame stepFrame = new PoseReferenceFrame("stepFrame", ReferenceFrame.getWorldFrame());
         stepFrame.setPoseAndUpdate(translation, rotation);

         QuadrantDependentList<FramePoint3D> contacts = new QuadrantDependentList<>();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            FramePoint3D point = new FramePoint3D(stepFrame, 0.5 * robotQuadrant.getEnd().negateIfHindEnd(nominalLength), 0.5 * robotQuadrant.getSide().negateIfRightSide(nominalWidth), 0.0);
            point.add(EuclidCoreRandomTools.nextVector3D(random, new Vector3D(0.5 * nominalLength, 0.5 * nominalWidth, 0.5 * nominalLength)));
            point.changeFrame(ReferenceFrame.getWorldFrame());
            contacts.put(robotQuadrant, point);
         }

         GroundPlaneEstimator groundPlaneEstimator = new GroundPlaneEstimator();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            groundPlaneEstimator.addContactPoint(contacts.get(robotQuadrant));
         groundPlaneEstimator.compute();

         double nominalYaw = PitchEstimator.computeNominalYaw(contacts);
         double expectedPitch = groundPlaneEstimator.getPitch(nominalYaw);
         double pitch = PitchEstimator.computeGroundPitchFromContacts(contacts);

         assertEquals(expectedPitch, pitch, 1e-5);
      }
   }
}

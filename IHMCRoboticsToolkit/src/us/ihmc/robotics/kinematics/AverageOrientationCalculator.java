package us.ihmc.robotics.kinematics;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Quat4d;

/**
 * @author twan
 *         Date: 5/10/13
 */
public class AverageOrientationCalculator
{
   private final FrameOrientation orientation1 = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final FrameOrientation orientation2 = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final FrameOrientation[] orientations = new FrameOrientation[] {orientation1, orientation2};

   private final Quat4d quaternion1 = new Quat4d();
   private final Quat4d quaternion2 = new Quat4d();
   private final Quat4d[] quaternions = new Quat4d[] {quaternion1, quaternion2};
   private final Quat4d averageQuaternion = new Quat4d();

   public void computeAverageOrientation(FrameOrientation orientationToPack, ReferenceFrame referenceFrame1, ReferenceFrame referenceFrame2, ReferenceFrame expressedInFrame)
   {
      orientation1.setToZero(referenceFrame1);
      orientation2.setToZero(referenceFrame2);

      for (int i = 0; i < orientations.length; i++)
      {
         FrameOrientation orientation = orientations[i];
         Quat4d quaternion = quaternions[i];
         orientation.changeFrame(expressedInFrame);
         orientation.getQuaternion(quaternion);
      }

      averageQuaternion.interpolate(quaternion1, quaternion2, 0.5);
      orientationToPack.setToZero(expressedInFrame);
      orientationToPack.set(averageQuaternion);
   }
}

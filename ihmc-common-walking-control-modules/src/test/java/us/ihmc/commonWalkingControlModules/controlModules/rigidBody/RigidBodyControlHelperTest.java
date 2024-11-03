package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class RigidBodyControlHelperTest
{
   private static final double epsilon = 1.0e-10;

   @Test
   public void testControlFrameChange()
   {
      Random random = new Random(328482L);

      for (int i = 0; i < 100; i++)
      {
         // Create a body frame and two control frames attached to the body.
         ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random, ReferenceFrame.getWorldFrame());
         FramePoint3D c1_origin = EuclidFrameRandomTools.nextFramePoint3D(random, bodyFrame);
         FrameQuaternion c1_orientation = EuclidFrameRandomTools.nextFrameQuaternion(random, bodyFrame);
         RigidBodyTransform c1ToBody = new RigidBodyTransform(c1_orientation, c1_origin);
         ReferenceFrame c1 = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("c1", bodyFrame, c1ToBody);
         FramePoint3D c2_origin = EuclidFrameRandomTools.nextFramePoint3D(random, bodyFrame);
         FrameQuaternion c2_orientation = EuclidFrameRandomTools.nextFrameQuaternion(random, bodyFrame);
         RigidBodyTransform c2ToBody = new RigidBodyTransform(c2_orientation, c2_origin);
         ReferenceFrame c2 = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("c2", bodyFrame, c2ToBody);

         // Create desired frames where c2_d is attached to c1_d using the constant transform between c1 and c2.
         RigidBodyTransform c2c1 = c2.getTransformToDesiredFrame(c1);
         PoseReferenceFrame c1_d = new PoseReferenceFrame("c1_d", ReferenceFrame.getWorldFrame());
         ReferenceFrame c2_d = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("c2_d", c1_d, c2c1);

         // Assert that with no tracking error c2_d will match c2.
         c1_d.setPoseAndUpdate(c1.getTransformToDesiredFrame(c1_d.getParent()));
         EuclidCoreTestTools.assertEquals(c1.getTransformToWorldFrame(), c1_d.getTransformToWorldFrame(), epsilon);
         EuclidCoreTestTools.assertEquals(c2.getTransformToWorldFrame(), c2_d.getTransformToWorldFrame(), epsilon);

         // Set c1_d to some desired pose then move the control frame and compute the expected desired for c2.
         FramePoint3D desiredPosition_c1 = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameQuaternion desiredOrientation_c1 = EuclidFrameRandomTools.nextFrameQuaternion(random, ReferenceFrame.getWorldFrame());
         c1_d.setPoseAndUpdate(desiredPosition_c1, desiredOrientation_c1);
         FramePoint3D expectedDesiredPosition_c2 = new FramePoint3D(c2_d);
         expectedDesiredPosition_c2.changeFrame(ReferenceFrame.getWorldFrame());
         FrameQuaternion expectedDesiredOrientation_c2 = new FrameQuaternion(c2_d);
         expectedDesiredOrientation_c2.changeFrame(ReferenceFrame.getWorldFrame());

         // Do the actual computation and assert the result is correct.
         FramePoint3D actualDesiredPosition_c2 = new FramePoint3D();
         FrameQuaternion actualDesiredOrienation_c2 = new FrameQuaternion();
         // Pose
         actualDesiredPosition_c2.setIncludingFrame(desiredPosition_c1);
         actualDesiredOrienation_c2.setIncludingFrame(desiredOrientation_c1);
         RigidBodyTransform tempC1ToBody = new RigidBodyTransform(c1ToBody);
         RigidBodyPositionControlHelper.modifyControlFrame(actualDesiredPosition_c2, actualDesiredOrienation_c2, tempC1ToBody, c2ToBody);
         RigidBodyOrientationControlHelper.modifyControlFrame(actualDesiredOrienation_c2, c1_orientation, c2_orientation);
         EuclidFrameTestTools.assertGeometricallyEquals(expectedDesiredPosition_c2, actualDesiredPosition_c2, epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(expectedDesiredOrientation_c2, actualDesiredOrienation_c2, epsilon);
         // Orientation
         actualDesiredOrienation_c2.setIncludingFrame(desiredOrientation_c1);
         RigidBodyOrientationControlHelper.modifyControlFrame(actualDesiredOrienation_c2, c1_orientation, c2_orientation);
         EuclidFrameTestTools.assertGeometricallyEquals(expectedDesiredOrientation_c2, actualDesiredOrienation_c2, epsilon);
      }
   }
}

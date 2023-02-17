package us.ihmc.perception.sensorHead;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class SensorHeadParameters
{
   public static final RigidBodyTransform OUSTER_TO_FISHEYE_TRANSFORM = new RigidBodyTransform();
   static
   {
      // For the benchtop sensorhead setup
      FramePose3D ousterPose = new FramePose3D();
      ousterPose.getPosition().set(0.225, 0.004, 0.459);
      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.setAndNormalize( 0.779, -0.155,  0.607,
                                      0.189,  0.982,  0.009,
                                     -0.598,  0.108,  0.794);
      ousterPose.getOrientation().set(rotationMatrix);
      ousterPose.getOrientation().appendPitchRotation(Math.toRadians(-2));

      RigidBodyTransform transformChestToBlackflyFujinon = new RigidBodyTransform();
      transformChestToBlackflyFujinon.setIdentity();
      transformChestToBlackflyFujinon.getTranslation().set(0.160, -0.095, 0.419);
      transformChestToBlackflyFujinon.getRotation().setAndNormalize( 0.986, -0.000, 0.167, 0.000, 1.000, -0.000, -0.167, 0.000, 0.986);
      ReferenceFrame blackflyFrame
            = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                       transformChestToBlackflyFujinon);

      ousterPose.changeFrame(blackflyFrame);
      ousterPose.get(OUSTER_TO_FISHEYE_TRANSFORM);
   }
}

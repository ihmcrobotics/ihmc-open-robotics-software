package us.ihmc.perception.sensorHead;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

/**
 * These parameters are specific to the sensor head setup we've designed for Nadia.
 * We have them here because they are used to develop and test robot agnostic algorithms,
 * which we use an off-board version of the head for.
 *
 * TODO: Possibly store different versions as StoredPropertySet and/or move to robot specific repository.
 */
public class SensorHeadParameters
{
   // These "for coloring" parameters are tuned by hand so the coloring on the
   // Ouster point cloud look better. It's probably not the correct model for doing
   // so, since the Ouster and Fisheye do not share the same origin. TODO Improve it
   public static final double FOCAL_LENGTH_X_FOR_COLORING = 472.44896;   // These were tuned with sliders on the benchtop
   public static final double FOCAL_LENGTH_Y_FOR_COLORING = 475.51022;   // by Bhavyansh and Duncan and copied here
   public static final double PRINCIPAL_POINT_X_FOR_COLORING = 970.06801;// by hand.
   public static final double PRINCIPAL_POINT_Y_FOR_COLORING = 608.84360;
   // Fujinon FE185C086HA_1 lens
   // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
   public static final double FE185C086HA_1_FOCAL_LENGTH = 0.0027;
   public static final double FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS
         = FE185C086HA_1_FOCAL_LENGTH * SpinnakerBlackfly.BFLY_U3_23S6C_WIDTH_PIXELS / SpinnakerBlackfly.BFLY_U3_23S6C_CMOS_SENSOR_WIDTH;

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

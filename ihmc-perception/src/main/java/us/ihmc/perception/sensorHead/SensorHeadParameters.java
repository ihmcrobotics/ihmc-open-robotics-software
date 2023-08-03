package us.ihmc.perception.sensorHead;

import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.robotics.EuclidCoreMissingTools;

/**
 * These parameters are specific to the sensor head setup we've designed for Nadia.
 * We have them here because they are used to develop and test robot agnostic algorithms,
 * which we use an off-board version of the head for.
 *
 * TODO: Possibly store different versions as StoredPropertySet and/or move to robot specific repository.
 */
public class SensorHeadParameters
{
   public static final BlackflyLensProperties BENCHTOP_BLACKFLY_LENS_COMBO = BlackflyLensProperties.BFS_U3_27S5C_FE185C086HA_1;

   // Fujinon FE185C086HA_1 lens
   // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
   public static final double FE185C086HA_1_FOCAL_LENGTH = 0.0027;

   /** We want to increase the resolution of the undistorted image to keep detail in the highly compacted center. */
   public static final double UNDISTORTED_IMAGE_SCALE = 1.6;
   /** Reduce the minimum allowable marker size (in pixels) so far away ones can be picked up. */
   public static final double ARUCO_MIN_MARKER_PERIMETER_RATE = 0.01;
   /** Helps with detecting blurry edges towards the edges of the fisheye's FOV. Wider edges. */
   public static final int ARUCO_ADAPTIVE_THRESHOLD_WINDOW_SIZE_MAX = 35;
   /** Helps with detecting blurry edges towards the edges of the fisheye's FOV. Trying more window sizes. */
   public static final int ARUCO_ADAPTIVE_THRESHOLD_WINDOW_SIZE_STEP = 6;
   /** Seems to increase robustness to varying lighting situations. Sometimes dark, sometimes bright.*/
   public static final double ARUCO_ADAPTIVE_THRESHOLD_CONSTANT = 19.3;
   /** Helps with detecting marker ID more consistently, higher number costs more computation though.
    *  It's the resolution of the rectified marker image for ID analysis. */
   public static final int PERSPECTIVE_REMOVE_PIXEL_PER_CELL = 10;
   /** Helps detect IDs more consistently, looks more at the center of each ArUco bit, discarding the ouster parts. */
   public static final double PERSPECTIVE_REMOVE_IGNORED_MARGIN_PER_CELL = 0.3;
   /** From CAD by @rharkins and @eyu */
   public static final double OUSTER_PITCH_ANGLE_DEGREES = 35.0;

   /**
    * This one represents the one on Nadia
    * Inverted y in respect to FISHEYE_RIGHT_TO_OUSTER_TRANSFORM_ON_ROBOT
    *
    * TODO: manually measure these
    */
   public static final RigidBodyTransform FISHEYE_LEFT_TO_OUSTER_TRANSFORM_ON_ROBOT = new RigidBodyTransform();
   static
   {
      FISHEYE_LEFT_TO_OUSTER_TRANSFORM_ON_ROBOT.getTranslation().set(-0.001668, 0.0675, -0.043698);
      Tuple3DReadOnly ousterBaseToBeamTranslation = getOusterBaseToBeamTranslation(OUSTER_PITCH_ANGLE_DEGREES);
      FISHEYE_LEFT_TO_OUSTER_TRANSFORM_ON_ROBOT.getTranslation().sub(ousterBaseToBeamTranslation);
      EuclidCoreMissingTools.setYawPitchRollDegrees(FISHEYE_LEFT_TO_OUSTER_TRANSFORM_ON_ROBOT.getRotation(), 0.000, -25.0, 0.000);
   }

   /**
    * This one represents the one on Nadia
    * These numbers taken from CAD by @eyu, and entered by @dcalvert.
    */
   public static final RigidBodyTransform FISHEYE_RIGHT_TO_OUSTER_TRANSFORM_ON_ROBOT = new RigidBodyTransform();
   static
   {
      FISHEYE_RIGHT_TO_OUSTER_TRANSFORM_ON_ROBOT.getTranslation().set(-0.001668, -0.0675, -0.043698);
      Tuple3DReadOnly ousterBaseToBeamTranslation = getOusterBaseToBeamTranslation(OUSTER_PITCH_ANGLE_DEGREES);
      FISHEYE_RIGHT_TO_OUSTER_TRANSFORM_ON_ROBOT.getTranslation().sub(ousterBaseToBeamTranslation);
      EuclidCoreMissingTools.setYawPitchRollDegrees(FISHEYE_RIGHT_TO_OUSTER_TRANSFORM_ON_ROBOT.getRotation(), 0.000, -25.0, 0.000);
   }

   /**
    * These "for coloring" parameters are tuned by hand so the coloring on the
    * Ouster point cloud look better. It's probably not the correct model for doing
    * so, since the Ouster and Fisheye do not share the same origin. TODO Improve it
    *
    * These were tuned with sliders on the benchtop by Bhavyansh and Duncan and copied here by hand.
    * ihmc-perception/src/main/resources/us/ihmc/perception/parameters/IntrinsicCameraMatrixPropertiesOusterFisheyeBenchtop.json
    */
   public static IntrinsicCameraMatrixProperties loadOusterFisheyeColoringIntrinsicsBenchtop()
   {
      return new IntrinsicCameraMatrixProperties("OusterFisheyeBenchtop");
   }

   /**
    * The BFLY parameters were tuned with sliders on the real Nadia robot with all sorts of objects out in the scene,
    * above and below and left and right of the camera prinicpal axis. by @dcalvert
    *
    * The Blackfly S 27S5C parameters were hand tuned by @dcalvert, @danderson, and @tbialek on 6/23/2023
    * with a setup with 4 ArUco markers out in the main lab space, one occupying each quadrant of vision.
    *
    * The JSON files can be found at:
    * ihmc-perception/src/main/resources/us/ihmc/perception/parameters/IntrinsicCameraMatrixProperties*.json
    */
   public static IntrinsicCameraMatrixProperties loadOusterFisheyeColoringIntrinsicsOnRobot(BlackflyLensProperties blackflyLensProperties)
   {
      return new IntrinsicCameraMatrixProperties(blackflyLensProperties.name());
   }

   public static void setArUcoMarkerDetectionParameters(DetectorParameters detectionParameters)
   {
      detectionParameters.minMarkerPerimeterRate(ARUCO_MIN_MARKER_PERIMETER_RATE);
      detectionParameters.adaptiveThreshWinSizeMax(ARUCO_ADAPTIVE_THRESHOLD_WINDOW_SIZE_MAX);
      detectionParameters.adaptiveThreshWinSizeStep(ARUCO_ADAPTIVE_THRESHOLD_WINDOW_SIZE_STEP);
      detectionParameters.adaptiveThreshConstant(ARUCO_ADAPTIVE_THRESHOLD_CONSTANT);
      detectionParameters.perspectiveRemovePixelPerCell(PERSPECTIVE_REMOVE_PIXEL_PER_CELL);
      detectionParameters.perspectiveRemoveIgnoredMarginPerCell(PERSPECTIVE_REMOVE_IGNORED_MARGIN_PER_CELL);
   }

   public static Tuple3DReadOnly getOusterBaseToBeamTranslation(double pitchAngleDegrees)
   {
      // https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
      double baseToBeamOS0 = 0.03618;
      // Ouster base to lidar sensor origin.
      return new Point3D(baseToBeamOS0 * Math.cos(Math.toRadians(90.0 - pitchAngleDegrees)),
                         0.0,
                         baseToBeamOS0 * Math.sin(Math.toRadians(90.0 - pitchAngleDegrees)));
   }
}

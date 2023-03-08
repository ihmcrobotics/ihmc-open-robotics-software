package us.ihmc.perception.sensorHead;

import org.bytedeco.opencv.opencv_aruco.DetectorParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
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
   // These "for coloring" parameters are tuned by hand so the coloring on the
   // Ouster point cloud look better. It's probably not the correct model for doing
   // so, since the Ouster and Fisheye do not share the same origin. TODO Improve it
   public static final double FOCAL_LENGTH_X_FOR_COLORING = 472.44896;   // These were tuned with sliders on the benchtop
   public static final double FOCAL_LENGTH_Y_FOR_COLORING = 449.76417;   // by Bhavyansh and Duncan and copied here
   public static final double PRINCIPAL_POINT_X_FOR_COLORING = 976.90677;// by hand.
   public static final double PRINCIPAL_POINT_Y_FOR_COLORING = 616.27357;
   // Fujinon FE185C086HA_1 lens
   // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
   public static final double FE185C086HA_1_FOCAL_LENGTH = 0.0027;
   public static final double FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS
         = FE185C086HA_1_FOCAL_LENGTH * SpinnakerBlackfly.BFLY_U3_23S6C_WIDTH_PIXELS / SpinnakerBlackfly.BFLY_U3_23S6C_CMOS_SENSOR_WIDTH;
   public static final double FOCAL_LENGTH_X_FOR_UNDISORTION = 451.18520;
   public static final double FOCAL_LENGTH_Y_FOR_UNDISORTION = 451.24202;
   public static final double PRINCIPAL_POINT_X_FOR_UNDISORTION = 969.78071;
   public static final double PRINCIPAL_POINT_Y_FOR_UNDISORTION = 609.84049;
   public static final double K1_FOR_UNDISORTION = 0.0066063;
   public static final double K2_FOR_UNDISORTION = 0.0103141;
   public static final double K3_FOR_UNDISORTION = -0.0056699;
   public static final double K4_FOR_UNDISORTION = 0.0007021;
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

   public static final RigidBodyTransform FISHEYE_TO_OUSTER_TRANSFORM = new RigidBodyTransform();
   static
   {
      FISHEYE_TO_OUSTER_TRANSFORM.getTranslation().set(0.002, -0.070, -0.077);
      EuclidCoreMissingTools.setYawPitchRollDegrees(FISHEYE_TO_OUSTER_TRANSFORM.getRotation(), 0.000, -25.0,  0.000);
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
}

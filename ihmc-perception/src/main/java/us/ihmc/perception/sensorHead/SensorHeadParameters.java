package us.ihmc.perception.sensorHead;

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
   public static final double FOCAL_LENGTH_X_FOR_UNDISORTION = 446.14308;
   public static final double FOCAL_LENGTH_Y_FOR_UNDISORTION = 446.15591;
   public static final double PRINCIPAL_POINT_X_FOR_UNDISORTION = 970.51657;
   public static final double PRINCIPAL_POINT_Y_FOR_UNDISORTION = 613.50150;
   public static final double K1_FOR_UNDISORTION = 0.0184961;
   public static final double K2_FOR_UNDISORTION = 0.0033343;
   public static final double K3_FOR_UNDISORTION = -0.0034367;
   public static final double K4_FOR_UNDISORTION = 0.0004191;

   public static final RigidBodyTransform FISHEYE_TO_OUSTER_TRANSFORM = new RigidBodyTransform();
   static
   {
      FISHEYE_TO_OUSTER_TRANSFORM.getTranslation().set(0.002, -0.070, -0.077);
      EuclidCoreMissingTools.setYawPitchRollDegrees(FISHEYE_TO_OUSTER_TRANSFORM.getRotation(), 0.000, -25.0,  0.000);
   }
}

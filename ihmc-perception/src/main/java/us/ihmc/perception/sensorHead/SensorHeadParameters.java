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
   public static final double FOCAL_LENGTH_Y_FOR_COLORING = 475.51022;   // by Bhavyansh and Duncan and copied here
   public static final double PRINCIPAL_POINT_X_FOR_COLORING = 970.06801;// by hand.
   public static final double PRINCIPAL_POINT_Y_FOR_COLORING = 608.84360;
   // Fujinon FE185C086HA_1 lens
   // https://www.fujifilm.com/us/en/business/optical-devices/machine-vision-lens/fe185-series
   public static final double FE185C086HA_1_FOCAL_LENGTH = 0.0027;
   public static final double FE185C086HA_1_FOCAL_LENGTH_IN_BFLY_U3_23S6C_PIXELS
         = FE185C086HA_1_FOCAL_LENGTH * SpinnakerBlackfly.BFLY_U3_23S6C_WIDTH_PIXELS / SpinnakerBlackfly.BFLY_U3_23S6C_CMOS_SENSOR_WIDTH;

   public static final RigidBodyTransform FISHEYE_TO_OUSTER_TRANSFORM = new RigidBodyTransform();
   static
   {
      FISHEYE_TO_OUSTER_TRANSFORM.getTranslation().set(0.005, -0.067, -0.077);
      EuclidCoreMissingTools.setYawPitchRollDegrees(FISHEYE_TO_OUSTER_TRANSFORM.getRotation(), 0.000, -24.369,  0.000);
   }
}

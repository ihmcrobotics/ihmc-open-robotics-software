package us.ihmc.avatar.continuousLocomotion;

public class AvatarWalkingModeManagerParameters
{
   private static final boolean assumeFlatGround = false;
   private static final boolean detectFlatGround = true;
   private static final boolean useInitialSupportRegions = true;

   private static final double assumedFlatGroundCircleRadius = 2.0;
   private static final double detectFlatGroundMinRadius = 1.5;
   private static final double detectFlatGroundMinRegionAreaToConsider = 0.03;
   private static final double detectFlatGroundOrientationTolerance = 0.1;
   private static final double detectFlatGroundZTolerance = 0.05;
   private static final double supportRegionScaleFactor = 3.0;

   private static final int planarRegionsHistorySize = 1;

   /**
    * If true, the footstep planning algorithm will assume flat ground and walk
    * regardless of sensor input, staying on the same plane and simply generating
    * footsteps on that plane. Only use if the robot has plenty of space and the
    * ground is very flat along the path you're taking.
    */
   public boolean getAssumeFlatGround()
   {
      return assumeFlatGround;
   }

   /**
    * If true, there will be virtual (not sensor based) flat regions introduced when
    * the sensor generated planar regions seem locally flat. This parameter is
    * included to help the robot turn in place and take tight corners when the sensor
    * field of view doesn't include the feet, but also because the feet are going to
    * always prevent clean flat regions beneath you. This is an option because it can
    * be a little risky depending on the environment.
    */
   public boolean getDetectFlatGround()
   {
      return detectFlatGround;
   }

   /**
    * If true, when look and step is initiated, it is given some support regions under
    * its feet, assuming it is currently supported. This helps the robot take its
    * first steps when the sensor can't see under the feet very well.
    */
   public boolean getUseInitialSupportRegions()
   {
      return useInitialSupportRegions;
   }

   /**
    * The max size of the generated circle when assuming flat ground. A big circle can
    * help the robot keep moving without pauses by allowing 2-3 step ahead plans.
    */
   public double getAssumedFlatGroundCircleRadius()
   {
      return assumedFlatGroundCircleRadius;
   }

   /**
    * If the closest non-coplanar region is beyond this radius around the robot,
    * it can be assumed that the robot is walking on flat ground.
    */
   public double getDetectFlatGroundMinRadius()
   {
      return detectFlatGroundMinRadius;
   }

   /**
    * Minimum planar region area required for a given planar region to be considered
    * in this algorithm. Planar regions smaller than this are considered insignificant
    * for example small flat rocks or bottle caps, etc.
    */
   public double getDetectFlatGroundMinRegionAreaToConsider()
   {
      return detectFlatGroundMinRegionAreaToConsider;
   }

   /**
    * This is used to determine if a given planar region is parallel with the foot's plane.
    * If the angle between the planar region and the foot sole's plane is below this value,
    * those two planes can be considered parallel with one another. This also applies when
    * determining if the planes of each foot are parallel with one another.
    */
   public double getDetectFlatGroundOrientationTolerance()
   {
      return detectFlatGroundOrientationTolerance;
   }

   /**
    * This is used to determine if a given planar region is at the same height as the
    * robot's foot. If the height (z) difference between the foot's plane and the planar
    * region is below this value, they can be assumed to be at the same height. This also
    * applies when determining if the planes of each foot are at the same height.
    */
   public double getDetectFlatGroundZTolerance()
   {
      return detectFlatGroundZTolerance;
   }

   /**
    * This is a scalar of the foot support polygons, used in the "Use initial support
    * regions" setting. Should be greater than 1. For example a value of 3 will give
    * the robot two big foot shaped regions, where it's feet are, that are scaled up
    * 3x, to assist with initial steps when the sensor can't see that area or the feet
    * are blocking it.
    */
   public double getSupportRegionScaleFactor()
   {
      return supportRegionScaleFactor;
   }

   /**
    * When set to 1, look and step will simply be using the latest set of planar
    * regions available. When set to n = 2+, the past n sets of planar regions from
    * successive scans of the environment will be merged together using the
    * PlanarRegionSLAM algorithm and the resulting map will be used for footstep
    * planning. A value of 0 means ignoring all planar regions from the sensor.
    */
   public int getPlanarRegionsHistorySize()
   {
      return planarRegionsHistorySize;
   }
}

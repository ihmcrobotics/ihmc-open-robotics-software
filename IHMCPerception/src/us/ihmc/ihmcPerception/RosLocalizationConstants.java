package us.ihmc.ihmcPerception;

/**
 * Created by dstephen on 3/20/15.
 */
public class RosLocalizationConstants
{
   public static final String OVERLAP_UPDATE_TOPIC = "/localization_overlap";
   public static final String STATUS_UPDATE_TOPIC = "/localization_status";
   public static final String POSE_UPDATE_TOPIC = "/icp_correction";
   public static final double DEFAULT_OVERLAP = 1.0;

   private RosLocalizationConstants(){}
}

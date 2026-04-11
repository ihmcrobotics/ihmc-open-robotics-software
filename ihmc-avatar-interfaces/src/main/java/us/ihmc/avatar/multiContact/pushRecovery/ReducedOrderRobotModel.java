package us.ihmc.avatar.multiContact.pushRecovery;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Reduced order robot model consisting of the centroidal and contact states.
 */
public class ReducedOrderRobotModel
{
   private static final double EPSILON = 1e-7;

   /* Nominal offset in mid-feet zup frame from CoM to shoulder position, taken at default standing home pose */
   public static final double SHOULDER_COM_OFFSET_X = -0.005;
   public static final double SHOULDER_COM_OFFSET_Y = 0.242;
   public static final double SHOULDER_COM_OFFSET_Z = 0.536;

   /* Minimum and maximum allowed reachablity distance from shoulder to hand */
   public static final double REACHABILITY_RADIUS_MIN = 0.32;
   public static final double REACHABILITY_RADIUS_MAX = 0.72;

   /* Maximum inward reaching distance, to prevent too much cross-over */
   public static final double MAX_INWARD_DISTANCE = SHOULDER_COM_OFFSET_Y * 0.9;

   public static final double NOMINAL_COM_HEIGHT = 0.9;
   public static final double OMEGA = Math.sqrt(9.81 / NOMINAL_COM_HEIGHT);

   public static boolean isReachable(RobotSide robotSide, FramePoint3DReadOnly queryPoint, ReferenceFrame centroidalFrame, FramePoint3D shoulderToContactPoint)
   {
      shoulderToContactPoint.setToZero(centroidalFrame);
      shoulderToContactPoint.setMatchingFrame(queryPoint);
      shoulderToContactPoint.sub(SHOULDER_COM_OFFSET_X, robotSide.negateIfRightSide(SHOULDER_COM_OFFSET_Y), SHOULDER_COM_OFFSET_Z);

      double shoulderToContactPointNorm = shoulderToContactPoint.norm();
      if (shoulderToContactPointNorm < REACHABILITY_RADIUS_MIN - EPSILON)
         return false;
      if (shoulderToContactPointNorm > REACHABILITY_RADIUS_MAX + EPSILON)
         return false;
      if (robotSide.negateIfRightSide(shoulderToContactPoint.getY()) < -MAX_INWARD_DISTANCE - EPSILON)
         return false;

      return true;
   }
}

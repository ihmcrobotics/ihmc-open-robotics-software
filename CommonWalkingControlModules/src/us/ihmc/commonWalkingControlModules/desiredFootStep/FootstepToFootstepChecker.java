package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/6/13
 * Time: 1:00 PM
 * To change this template use File | Settings | File Templates.
 */
public class FootstepToFootstepChecker
{
   private static final boolean DEBUG = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static FramePose startingFramePose = new FramePose(worldFrame);
   private static FramePose endingFramePose = new FramePose(worldFrame);

   private static double MAXIMUM_DELTA_R = 0.6;
   private static double MAXIMUM_DELTA_R_SQUARED = MAXIMUM_DELTA_R * MAXIMUM_DELTA_R;
   private static double MAXIMUM_DELTA_Z = 0.3;

   private static double MAXIMUM_DELTA_ROLL = Math.toRadians(15.0);    // 0.26
   private static double MAXIMUM_DELTA_PITCH = Math.toRadians(15.0);    // 0.26
   private static double MAXIMUM_DELTA_YAW = Math.toRadians(90.0);    // 1.57 radians
   
   private static final double semiCircleOffset = 0.14; //From RangeOfStepGraphicsManager: Should have a unified source, not just a magic number.
   
         public static boolean isFootstepToFootstepChangeLarge(Footstep startingFootstep, Footstep endingFootstep, RobotSide startingFootstepSide)
   {
      startingFootstep.getPose(startingFramePose);
      endingFootstep.getPose(endingFramePose);

      FrameVector startToEnd = new FrameVector(endingFramePose.getPositionCopy());
      startToEnd.sub(startingFramePose.getPositionCopy());

      // TODO: Best would be to base on kinematic and dynamic reachability (which would be robot dependent and not so much magic tuned numbers)
      // Would also catch unspecified kinematic coupling (e.g. coupling that may occur based on yaw and step height and others?)

      // TODO: Make max values consistent with other values used in other files
      // The maximum delta pitch/roll are smaller than risk regions for a single raw step (see DRCOperatorInterfaceConfig)
      // defaultMaxPitchRiskColor;
      // rollSlope  -> 0.28 - (-0.28) = 0.56
      // pitchSlope -> 0.32 - (-0.5)  = 0.82
      //
      // defaultHighPitchRiskColor;
      // rollSlope  -> 0.22 - (-0.22) = 0.44
      // pitchSlope -> 0.25 - (-0.3)  = 0.55
      //
      // defaultMidPitchRiskColor;
      // rollSlope  -> 0.15 - (-0.15) = 0.30
      // pitchSlope -> 0.15 - (-0.2)  = 0.35
      //
      // defaultLowPitchRiskColor;
      // rollSlope  -> 0.15 - (-0.15) = 0.30
      // pitchSlope -> 0.15 - (-0.15) = 0.30
      // For yaw, turning footstep generators use an opening and a closing angle:
      // StraightLinePath (values used for creating steps):
      // private double maximumHipOpeningAngle = Math.toRadians(10.0);
      // private double maximumHipClosingAngle = Math.toRadians(5.0);
      // DRCRobotBasedFootstepGeneratorTest (used only in this test)
      // footstepGenerator.setTurningStepsHipOpeningStepAngle(Math.PI / 6);//15 degrees
      // TurningThenStraightFootstepGenerator default (never used except maybe in Test above):
      // turningWalkingOpeningAngleIncrement.set(Math.PI * 0.8);//144 degrees
      // turningWalkingClosingAngleIncrement.set(Math.PI * 0.15);// 27 degrees
      // This has equal X and Y dimensions, but if allowable distance is direction dependent, then it needs to be a different shape (e.g. elliptical).
      // This is in global coordinates, but should be in the stance foot (startingFootstep) frame if changed to non-circular.
      // SemiCircularStepValidityMetric does something similar to this for tests. It suggests a 1.2 rad max (35.42 deg) and 1.5m radius which may both be arbitrary

      // TODO: This function will not always notify of values that are "Outside allowable range" resulting in an 'OutsideSerializableValuesException' when sending footsteps.
      // us.ihmc.utilities.fixedPointRepresentation.OutsideSerializableValuesException
      // From FootstepDataListSerializer:
      // public static final double[] XYZ_MAX = { 1.0, 1.0, 0.5 };
      // public static final double[] XYZ_MIN = { -1.0, -1.0, -1.5 };
      // This range is based on subsequent footsteps, not stance/step pairs. So this is most likely violated when making large steps with the same side to opposite extremes
      // For example -0.6 to 0.6 is a 1.2 difference which is larger than the allowable serializable range. This is unlikely to be done in practice.

      // TODO: deltaStepDirection to enforce semicircular constraint, but it is too stringent. Some valid steps will be marked red, but
      // validity depends on trajectory and final position kinematics (prior step of swinging step: Need to make sure it won't pass through stance leg).
      // Also, steps on other half are kinematically more challenging. It is safer to just mark all footsteps on stance side of semi-cicle as invalid until
      // better way determined based on kinematics (reachability), swing trajectory, self collision, and possibly dynamic feasability.

      double deltaX = Math.abs(startToEnd.getX());
      double deltaY = Math.abs(startToEnd.getY());
      double deltaZ = Math.abs(startToEnd.getZ());
      double deltaRSquared = deltaX * deltaX + deltaY * deltaY;

      double deltaYaw = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getYaw(), startingFramePose.getYaw()));
      double deltaPitch = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getPitch(), startingFramePose.getPitch()));
      double deltaRoll = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getRoll(), startingFramePose.getRoll()));

      startToEnd = new FrameVector(endingFramePose.getPositionCopy());
      FrameVector offsetStart = new FrameVector(startingFramePose.getPositionCopy());
      double sideOffset = semiCircleOffset*(startingFootstepSide == RobotSide.LEFT ? -1: 1);
      double offsetDirAngle = startingFramePose.getYaw() + Math.PI/2;
      FrameVector offset = new FrameVector(worldFrame,new double[] {Math.cos(offsetDirAngle)*sideOffset,Math.sin(offsetDirAngle)*sideOffset,0});
      offsetStart.add(offset);
      startToEnd.sub(offsetStart);
      double stepDirection = Math.atan2(startToEnd.getY(), startToEnd.getX());
      double deltaStepDirection = AngleTools.computeAngleDifferenceMinusPiToPi(stepDirection, startingFramePose.getYaw());

      boolean ret;

      if (deltaRSquared > MAXIMUM_DELTA_R_SQUARED)
         ret = true;
      else if (deltaZ > MAXIMUM_DELTA_Z)
         ret = true;
      else if (deltaYaw > MAXIMUM_DELTA_YAW)
         ret = true;
      else if (deltaPitch > MAXIMUM_DELTA_PITCH)
         ret = true;
      else if (deltaRoll > MAXIMUM_DELTA_ROLL)
         ret = true;
      else if (startingFootstepSide == RobotSide.LEFT ? deltaStepDirection > 0: deltaStepDirection < 0)
         ret = true;
      else
         ret = false;

      printIfDebug(ret, deltaX, deltaY, deltaZ, deltaRSquared, deltaYaw, deltaPitch, deltaRoll);

      return ret;
   }

   public static double getFootstepPitchSlope(Footstep footstep)
   {
      FrameVector frameVector = new FrameVector(footstep.getPoseReferenceFrame(), 1.0, 0.0, 0.0);
      frameVector.changeFrame(ReferenceFrame.getWorldFrame());
      printIfDebug("frameVector = " + frameVector);

      double pitchSlope = frameVector.getZ();

      printIfDebug("pitchSlope = " + pitchSlope);

      return pitchSlope;
   }

   public static double getFootstepRollSlope(Footstep footstep)
   {
      FrameVector frameVector = new FrameVector(footstep.getPoseReferenceFrame(), 0.0, 1.0, 0.0);
      frameVector.changeFrame(ReferenceFrame.getWorldFrame());
      printIfDebug("frameVector = " + frameVector);

      double rollSlope = frameVector.getZ();

      printIfDebug("rollSlope = " + rollSlope);

      return rollSlope;
   }

   private static void printIfDebug(String message)
   {
      if (DEBUG)
      {
         System.out.println(message);
      }
   }

   private static void printIfDebug(boolean ret, double x, double y, double z, double r2, double yaw, double pitch, double roll)
   {
      if (!DEBUG)
         return;

      if (ret)
         System.err.println("FootstepToFootstepChecker: " + ret + ", x" + x + ", y" + y + ", z" + z + ", r" + Math.sqrt(r2) + ", y" + yaw + ", p" + pitch
                            + ", r" + roll);
      else
         System.out.println("FootstepToFootstepChecker: " + ret + ", x" + x + ", y" + y + ", z" + z + ", r" + Math.sqrt(r2) + ", y" + yaw + ", p" + pitch
                            + ", r" + roll);
   }
}

package us.ihmc.commonWalkingControlModules.desiredFootStep;

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
   private static double MAXIMUM_DELTA_R_SQUARED = MAXIMUM_DELTA_R*MAXIMUM_DELTA_R;
   private static double MAXIMUM_DELTA_Z = 0.3;

   //TODO: The maximum delta pitch/roll are smaller than risk regions for a single raw step (see DRCOperatorInterfaceConfig)  
   private static double MAXIMUM_ROLL = Math.toRadians(15.0);//0.26
   private static double MAXIMUM_PITCH = Math.toRadians(15.0);
   private static double MAXIMUM_YAW = Math.toRadians(90.0);//1.57

   public static boolean isFootstepToFootstepChangeLarge(Footstep startingFootstep, Footstep endingFootstep)
   {
      startingFootstep.getPose(startingFramePose);
      endingFootstep.getPose(endingFramePose);

      FrameVector startToEnd = new FrameVector(endingFramePose.getPositionCopy());
      startToEnd.sub(startingFramePose.getPositionCopy());

      //TODO: This has equal X and Y dimensions, but if allowable distance is direction dependent, then it needs to be a different shape (e.g. elliptical).
      //      This is in global coordinates, but should be in the stance foot (startingFootstep) frame if changed to non-circular.
      //TODO: This function will not always notify of values that are "Outside allowable range" resulting in an 'OutsideSerializableValuesException' when sending footsteps.
      //       us.ihmc.utilities.fixedPointRepresentation.OutsideSerializableValuesException
      //     From FootstepDataListSerializer:
      //      public static final double[] XYZ_MAX = { 1.0, 1.0, 0.5 };
      //      public static final double[] XYZ_MIN = { -1.0, -1.0, -1.5 };
      //     This range is based on subsequent footsteps, not stance/step pairs. So this is most likely violated when making large steps with the same side to opposite extremes
      //      For example -0.6 to 0.6 is a 1.2 difference which is larger than the allowable serializable range. This is unlikely to be done in practice.

      double deltaX = Math.abs(startToEnd.getX());
      double deltaY = Math.abs(startToEnd.getY());
      double deltaZ = Math.abs(startToEnd.getZ());

      double deltaRSquared = deltaX*deltaX + deltaY*deltaY;
      
      double deltaYaw = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getYaw(), startingFramePose.getYaw()));
      double deltaPitch = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getPitch(), startingFramePose.getPitch()));
      double deltaRoll = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getRoll(), startingFramePose.getRoll()));
      
      boolean ret;

      if (deltaRSquared > MAXIMUM_DELTA_R_SQUARED)
         ret = true;
      else if (deltaZ > MAXIMUM_DELTA_Z)
         ret = true;
      else if (deltaYaw > MAXIMUM_YAW)
         ret = true;
      else if (deltaPitch > MAXIMUM_PITCH)
         ret = true;
      else if (deltaRoll > MAXIMUM_ROLL)
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
         System.err.println("FootstepToFootstepChecker: " + ret + ", x" + x + ", y" + y + ", z" + z + ", r" + Math.sqrt(r2) + ", y" + yaw + ", p" + pitch + ", r" + roll);
      else
         System.out.println("FootstepToFootstepChecker: " + ret + ", x" + x + ", y" + y + ", z" + z + ", r" + Math.sqrt(r2) + ", y" + yaw + ", p" + pitch + ", r" + roll);
   }

   

}

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.media.j3d.Transform3D;

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

   private static double MAXIMUM_DELTA_X = 1.0;
   private static double MAXIMUM_DELTA_Y = 1.0;
   private static double MAXIMUM_DELTA_Z = 0.3;

   private static double MAXIMUM_ROLL = Math.toRadians(30.0);
   private static double MAXIMUM_PITCH = Math.toRadians(30.0);
   private static double MAXIMUM_YAW = Math.toRadians(60.0);


   public static boolean isFootstepToFootstepChangeLarge(Footstep startingFootstep, Footstep endingFootstep)
   {
      setFramePose(startingFramePose, startingFootstep);
      setFramePose(endingFramePose, endingFootstep);

      FrameVector startToEnd = new FrameVector(endingFramePose.getPositionCopy());
      startToEnd.sub(startingFramePose.getPositionCopy());


      double deltaX = Math.abs(startToEnd.getX());
      double deltaY = Math.abs(startToEnd.getY());
      double deltaZ = Math.abs(startToEnd.getZ());

      double deltaYaw = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getYaw(), startingFramePose.getYaw()));
      double deltaPitch = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getPitch(), startingFramePose.getPitch()));
      double deltaRoll = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(endingFramePose.getRoll(), startingFramePose.getRoll()));

      boolean ret;

      if (deltaX > MAXIMUM_DELTA_X)
         ret = true;
      else if (deltaY > MAXIMUM_DELTA_Y)
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

      printIfDebug(ret, deltaX, deltaY, deltaZ, deltaYaw, deltaPitch, deltaRoll);

      return ret;
   }

   private static void printIfDebug(boolean ret, double x, double y, double z, double yaw, double pitch, double roll)
   {
      if (!DEBUG)
         return;

      System.out.println("FootstepToFootstepChecker: " + ret + ", " + x + ", " + y + ", " + z + ", " + yaw + ", " + pitch + ", " + roll);
   }

   private static void setFramePose(FramePose framePose, Footstep footstep)
   {
      ContactablePlaneBody endEffector = footstep.getBody();
      Transform3D soleTransform = endEffector.getPlaneFrame().getTransformToDesiredFrame(worldFrame);
      framePose.set(worldFrame, soleTransform);
   }
}

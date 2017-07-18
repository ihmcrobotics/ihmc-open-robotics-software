package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.yoVariables.variable.YoDouble;

public class CenterOfMassTrajectoryHandler
{
   private final YoDouble yoTime;

   public CenterOfMassTrajectoryHandler(YoDouble yoTime)
   {
      this.yoTime = yoTime;
   }

   public void handleComTrajectory(CenterOfMassTrajectoryCommand command)
   {
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the current controller time. If the handler does not have
    * a valid trajectory for this time the method will return false and set the desireds to be NaN.
    *
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    *
    * @return whether a valid trajectory point for this time was available
    *
    * @see CenterOfMassTrajectoryHandler#packDesiredICPAtTime(double, double, FramePoint, FrameVector)
    */
   public boolean packCurrentDesiredICP(double omega0, FramePoint desiredICPPositionToPack, FrameVector desiredICPVelocityToPack)
   {
      return packDesiredICPAtTime(yoTime.getDoubleValue(), omega0, desiredICPPositionToPack, desiredICPVelocityToPack);
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the provided controller time. If the handler does not have
    * a valid trajectory for this time the method will return false and set the desireds to be NaN.
    *
    * @param controllerTime
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    *
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packDesiredICPAtTime(double controllerTime, double omega0, FramePoint desiredICPPositionToPack, FrameVector desiredICPVelocityToPack)
   {
      return false;
   }

}

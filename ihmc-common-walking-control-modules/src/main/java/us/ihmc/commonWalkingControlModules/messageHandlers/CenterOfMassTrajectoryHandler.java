package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CenterOfMassTrajectoryHandler extends EuclideanTrajectoryHandler
{
   private final Point3D icpPosition = new Point3D();
   private final Vector3D icpVelocity = new Vector3D();

   private final FrameVector3D offset = new FrameVector3D();

   public CenterOfMassTrajectoryHandler(YoDouble yoTime, YoRegistry parentRegistry)
   {
      super("CenterOfMass", yoTime, parentRegistry);
   }

   public void handleComTrajectory(CenterOfMassTrajectoryCommand command)
   {
      handleTrajectory(command.getEuclideanTrajectory());
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the current controller time. If the handler does not
    * have a valid trajectory for this time the method will return false and set the desireds to be NaN.
    *
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packCurrentDesiredICP(double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack)
   {
      return packCurrentDesiredICP(omega0, desiredICPPositionToPack, desiredICPVelocityToPack, null);
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the current controller time. If the handler does not
    * have a valid trajectory for this time the method will return false and set the desireds to be NaN. This method also
    * packs the current desired center of mass position for visualization.
    *
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @param comPositionToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packCurrentDesiredICP(double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack,
                                        FramePoint3D comPositionToPack)
   {
      return packDesiredICPAtTime(getCurrentTime(), omega0, desiredICPPositionToPack, desiredICPVelocityToPack, comPositionToPack);
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the provided controller time. If the handler does not
    * have a valid trajectory for this time the method will return false and set the desireds to be NaN.
    *
    * @param controllerTime
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packDesiredICPAtTime(double controllerTime, double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack)
   {
      return packDesiredICPAtTime(controllerTime, omega0, desiredICPPositionToPack, desiredICPVelocityToPack, null);
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the provided controller time. If the handler does not
    * have a valid trajectory for this time the method will return false and set the desireds to be NaN. This method also
    * packs the current desired center of mass position for visualization.
    *
    * @param controllerTime
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @param comPositionToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packDesiredICPAtTime(double controllerTime, double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack,
                                       FramePoint3D comPositionToPack)
   {
      if (!isWithinInterval(controllerTime))
      {
         desiredICPPositionToPack.setToNaN(ReferenceFrame.getWorldFrame());
         desiredICPVelocityToPack.setToNaN(ReferenceFrame.getWorldFrame());
         if (comPositionToPack != null)
         {
            comPositionToPack.setToNaN(ReferenceFrame.getWorldFrame());
         }
         return false;
      }

      packDesiredsAtTime(controllerTime);
      icpPosition.scaleAdd(1.0 / omega0, getVelocity(), getPosition());
      icpVelocity.scaleAdd(1.0 / omega0, getAcceleration(), getVelocity());

      desiredICPPositionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), icpPosition);
      desiredICPVelocityToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), icpVelocity);
      desiredICPPositionToPack.add(offset);

      if (comPositionToPack != null)
      {
         comPositionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), getPosition());
         comPositionToPack.add(offset);
      }
      return true;
   }

   public void setPositionOffset(FrameVector3DReadOnly offset)
   {
      this.offset.setIncludingFrame(offset);
   }
}

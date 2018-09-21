package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MomentumTrajectoryHandler extends EuclideanTrajectoryHandler
{
   public MomentumTrajectoryHandler(YoDouble yoTime, YoVariableRegistry parentRegistry)
   {
      super("AngularMomentum", yoTime, parentRegistry);
   }

   public void handleMomentumTrajectory(MomentumTrajectoryCommand command)
   {
      handleTrajectory(command.getAngularMomentumTrajectory());
   }

   public void getAngularMomentumTrajectory(double startTime, double endTime, int numberOfPoints,
                                            RecyclingArrayList<SimpleEuclideanTrajectoryPoint> trajectoryToPack)
   {
      trajectoryToPack.clear();
      if (!isWithinInterval(startTime) || !isWithinInterval(endTime))
      {
         return;
      }

      for (int idx = 0; idx < numberOfPoints; idx++)
      {
         double time = startTime + (endTime - startTime) * idx / (numberOfPoints - 1);
         packDesiredsAtTime(time);

         SimpleEuclideanTrajectoryPoint trajectoryPoint = trajectoryToPack.add();
         trajectoryPoint.setTime(time - startTime);
         trajectoryPoint.setPosition(getPosition());
         trajectoryPoint.setLinearVelocity(getVelocity());
      }
   }

   public boolean packDesiredAngularMomentumAtTime(double time, FrameVector3DBasics angularMomentumToPack, FrameVector3DBasics angularMomentumRateToPack)
   {
      if (!isWithinInterval(time))
      {
         angularMomentumToPack.setToNaN(ReferenceFrame.getWorldFrame());
         angularMomentumRateToPack.setToNaN(ReferenceFrame.getWorldFrame());
         return false;
      }

      packDesiredsAtTime(time);

      angularMomentumToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), getPosition());
      angularMomentumRateToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), getVelocity());
      return true;
   }
}

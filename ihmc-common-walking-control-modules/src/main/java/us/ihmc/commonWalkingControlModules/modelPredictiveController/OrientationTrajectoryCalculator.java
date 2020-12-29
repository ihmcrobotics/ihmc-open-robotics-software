package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class OrientationTrajectoryCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final MultipleWaypointsOrientationTrajectoryGenerator trajectory;
   private final YoFrameQuaternion initialBodyOrientation = new YoFrameQuaternion("initialBodyOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D initialBodyAngularVelocity = new YoFrameVector3D("initialBodyAngularVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();

   public OrientationTrajectoryCalculator(YoRegistry parentRegistry)
   {
      trajectory = new MultipleWaypointsOrientationTrajectoryGenerator("bodyOrientation", ReferenceFrame.getWorldFrame(), registry);
      parentRegistry.addChild(registry);
   }

   public void setInitialBodyOrientation(FrameOrientation3DReadOnly initialBodyOrientation, FrameVector3DReadOnly initialBodyAngularVelocity)
   {
      this.initialBodyOrientation.set(initialBodyOrientation);
      this.initialBodyAngularVelocity.set(initialBodyAngularVelocity);
   }

   private final FrameQuaternion nominalOrientation = new FrameQuaternion();

   private final FrameQuaternion orientationSetpoint = new FrameQuaternion();
   private final FrameVector3D velocitySetpoint = new FrameVector3D();

   public void solveForTrajectory(List<ContactPlaneProvider> fullContactSequence)
   {
      trajectory.clear();

      nominalOrientation.set(initialBodyOrientation);
      velocitySetpoint.set(initialBodyAngularVelocity);

      trajectory.appendWaypoint(fullContactSequence.get(0).getTimeInterval().getStartTime(), nominalOrientation, velocitySetpoint);

      velocitySetpoint.setToZero();

      for (int i = 0; i < fullContactSequence.size(); i++)
      {
         ContactPlaneProvider contact = fullContactSequence.get(i);
         if (contact.getNumberOfContactPlanes() > 1)
            nominalOrientation.interpolate(contact.getContactPose(0).getOrientation(), contact.getContactPose(1).getOrientation(), 0.5);
         else if (contact.getNumberOfContactPlanes() > 0)
            nominalOrientation.set(contact.getContactPose(0).getOrientation());

         orientationSetpoint.setToYawOrientation(nominalOrientation.getYaw());
         trajectory.appendWaypoint(contact.getTimeInterval().getEndTime(), orientationSetpoint, velocitySetpoint);
      }
   }

   public void compute(double time)
   {
      trajectory.compute(time);

      trajectory.getOrientation(desiredOrientation);
      trajectory.getAngularVelocity(desiredAngularVelocity);
   }

   public FrameOrientation3DReadOnly getDesiredOrientation()
   {
      return desiredOrientation;
   }

   public FrameVector3DReadOnly getDesiredAngularVelocity()
   {
      return desiredAngularVelocity;
   }
}

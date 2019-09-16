package us.ihmc.stateEstimation.head;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;

public class SimulatedIMU
{
   private final RigidBodyBasics imuBody;
   private final MovingReferenceFrame imuFrame;
   private final SpatialAccelerationCalculator accelerationCalculator;
   private final SpatialAcceleration imuAcceleration = new SpatialAcceleration();
   private final FramePoint3D imuLocationOnBody = new FramePoint3D();
   private final FrameVector3D velocityCrossTerm = new FrameVector3D();

   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();

   public SimulatedIMU(MovingReferenceFrame imuFrame, RigidBodyBasics imuBody)
   {
      this.imuFrame = imuFrame;
      this.imuBody = imuBody;
      imuLocationOnBody.setToZero(imuFrame);
      imuLocationOnBody.changeFrame(imuBody.getBodyFixedFrame());
      accelerationCalculator = new SpatialAccelerationCalculator(imuBody, ReferenceFrame.getWorldFrame());
      accelerationCalculator.setGravitionalAcceleration(RobotState.GRAVITY);
   }

   public void compute()
   {
      accelerationCalculator.reset();
      angularVelocity.setIncludingFrame(imuFrame.getTwistOfFrame().getAngularPart());
      imuAcceleration.setIncludingFrame(accelerationCalculator.getAccelerationOfBody(imuBody));
      imuAcceleration.changeFrame(imuFrame);
      linearAcceleration.setIncludingFrame(imuAcceleration.getLinearPart());
      velocityCrossTerm.setToZero(imuFrame);
      velocityCrossTerm.cross(imuFrame.getTwistOfFrame().getAngularPart(), imuFrame.getTwistOfFrame().getLinearPart());
      linearAcceleration.add(velocityCrossTerm);
      orientation.setToZero(imuFrame);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public FrameQuaternionReadOnly getOrientation()
   {
      return orientation;
   }

   public FrameVector3DReadOnly getAngularVelocity()
   {
      return angularVelocity;
   }

   public FrameVector3DReadOnly getLinearAcceleration()
   {
      return linearAcceleration;
   }
}

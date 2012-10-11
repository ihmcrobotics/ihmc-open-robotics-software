package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.controlModules.SE3PDController;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class RigidBodySpatialAccelerationControlModule
{
   protected final YoVariableRegistry registry;
   protected final ReferenceFrame elevatorFrame;
   private final TwistCalculator twistCalculator;
   private final SE3PDController se3pdController;
   protected final SpatialAccelerationVector acceleration;
   protected final RigidBody endEffector;
   protected final ReferenceFrame endEffectorFrame;

   public RigidBodySpatialAccelerationControlModule(String namePrefix, ReferenceFrame elevatorFrame, TwistCalculator twistCalculator, RigidBody endEffector,
           ReferenceFrame endEffectorFrame, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.elevatorFrame = elevatorFrame;
      this.twistCalculator = twistCalculator;
      this.endEffector = endEffector;
      this.endEffectorFrame = endEffectorFrame;
      this.se3pdController = new SE3PDController(namePrefix, endEffectorFrame, registry);
      this.acceleration = new SpatialAccelerationVector(endEffectorFrame, elevatorFrame, endEffectorFrame);
      parentRegistry.addChild(registry);
   }

   public void packAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(acceleration);
   }

   public FrameVector getPositionErrorInWorld()
   {
      FrameVector ret = new FrameVector(endEffectorFrame);
      se3pdController.getPositionController().packPositionError(ret);
      ret.changeFrame(ReferenceFrame.getWorldFrame());

      return ret;
   }

   public void doPositionControl(FramePose desiredEndEffectorPose, Twist desiredEndEffectorTwist,
                                 SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration)
   {
      Twist currentTwist = new Twist();
      twistCalculator.packTwistOfBody(currentTwist, endEffector);
      currentTwist.changeBodyFrameNoRelativeTwist(endEffectorFrame);
      currentTwist.changeFrame(endEffectorFrame);
      currentTwist.changeBaseFrameNoRelativeTwist(elevatorFrame);

      se3pdController.compute(acceleration, desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, currentTwist);
   }

   public void setPositionProportionalGains(double kx, double ky, double kz)
   {
      se3pdController.getPositionController().setProportionalGains(kx, ky, kz);
   }

   public void setPositionDerivativeGains(double bx, double by, double bz)
   {
      se3pdController.getPositionController().setDerivativeGains(bx, by, bz);
   }

   public void setOrientationProportionalGains(double kx, double ky, double kz)
   {
      se3pdController.getOrientationController().setProportionalGains(kx, ky, kz);
   }

   public void setOrientationDerivativeGains(double bx, double by, double bz)
   {
      se3pdController.getOrientationController().setDerivativeGains(bx, by, bz);
   }
}

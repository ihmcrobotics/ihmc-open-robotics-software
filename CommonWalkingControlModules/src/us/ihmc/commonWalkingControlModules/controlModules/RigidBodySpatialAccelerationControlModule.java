package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class RigidBodySpatialAccelerationControlModule
{
   private final YoVariableRegistry registry;
   private final TwistCalculator twistCalculator;
   private final SE3PDController se3pdController;
   private final SpatialAccelerationVector acceleration;
   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;
   
   private final YoFrameVector desiredAccelerationLinearViz, desiredAccelerationAngularViz;

   private final DoubleYoVariable maximumLinearAccelerationMagnitude, maximumAngularAccelerationMagnitude;
   private final BooleanYoVariable limitAccelerations;
   
   public RigidBodySpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, twistCalculator, endEffector, endEffectorFrame, false, parentRegistry);
   }
   
   public RigidBodySpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame,
           boolean visualize, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.twistCalculator = twistCalculator;
      this.endEffector = endEffector;
      this.endEffectorFrame = endEffectorFrame;
      this.se3pdController = new SE3PDController(namePrefix, endEffectorFrame, visualize, registry);
      this.acceleration = new SpatialAccelerationVector();
      
      desiredAccelerationLinearViz = new YoFrameVector(namePrefix + "LinearAccelViz", endEffectorFrame, registry);
      desiredAccelerationAngularViz = new YoFrameVector(namePrefix + "AngularAccelViz", endEffectorFrame, registry);
      
      maximumLinearAccelerationMagnitude = new DoubleYoVariable(namePrefix + "MaxLinearAccelMagnitude", registry); 
      maximumAngularAccelerationMagnitude = new DoubleYoVariable(namePrefix + "MaxAngularAccelMagnitude", registry); 
      limitAccelerations = new BooleanYoVariable(namePrefix + "LimitAccelerations", registry);
      limitAccelerations.set(false);
      
      parentRegistry.addChild(registry);
   }
   
   public void setMaximumLinearAccelerationMagnitude(double maximumLinearAccelerationMagnitude)
   {
      this.maximumLinearAccelerationMagnitude.set(maximumLinearAccelerationMagnitude);
   }
   
   public void setMaximumAngularAccelerationMagnitude(double maximumAngularAccelerationMagnitude)
   {
      this.maximumAngularAccelerationMagnitude.set(maximumAngularAccelerationMagnitude);
   }
   
   public void setLimitAccelerations(boolean limitAccelerations)
   {
      this.limitAccelerations.set(limitAccelerations);
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
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
                                 SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration, RigidBody base)
   {
      Twist currentTwist = new Twist();
      twistCalculator.packRelativeTwist(currentTwist, base, endEffector);
      currentTwist.changeBodyFrameNoRelativeTwist(endEffectorFrame);
      currentTwist.changeFrame(endEffectorFrame);

      se3pdController.compute(acceleration, desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, currentTwist);
      
      
      if (limitAccelerations.getBooleanValue())
      {
         acceleration.limitLinearPartMagnitude(maximumLinearAccelerationMagnitude.getDoubleValue());
         acceleration.limitAngularPartMagnitude(maximumAngularAccelerationMagnitude.getDoubleValue());
      }
      
      acceleration.getExpressedInFrame().checkReferenceFrameMatch(desiredAccelerationLinearViz.getReferenceFrame());

      desiredAccelerationLinearViz.set(acceleration.getLinearPartCopy());
      desiredAccelerationAngularViz.set(acceleration.getAngularPartCopy());
   }

   public void doPositionControl(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocityOfOrigin,
                                 FrameVector desiredAngularVelocity, FrameVector desiredLinearAccelerationOfOrigin, FrameVector desiredAngularAcceleration,
                                 RigidBody base)
   {
      FramePose desiredEndEffectorPose = calculateDesiredEndEffectorPoseFromDesiredPositions(desiredPosition, desiredOrientation);
      Twist desiredEndEffectorTwist = calculateDesiredEndEffectorTwist(desiredLinearVelocityOfOrigin, desiredAngularVelocity, base);
      SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration =
         calculateDesiredEndEffectorSpatialAcceleration(desiredLinearAccelerationOfOrigin, desiredAngularAcceleration, base);
      doPositionControl(desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, base);
   }

   public FramePose calculateDesiredEndEffectorPoseFromDesiredPositions(FramePoint endEffectorPositionIn, FrameOrientation endEffectorOrientationIn)
   {
      FramePoint endEffectorPosition = endEffectorPositionIn.changeFrameCopy(endEffectorFrame);
      FrameOrientation endEffectorOrientation = endEffectorOrientationIn.changeFrameCopy(endEffectorFrame);

      return new FramePose(endEffectorPosition, endEffectorOrientation);
   }

   public Twist calculateDesiredEndEffectorTwist(FrameVector linearVelocityOfOrigin, FrameVector angularVelocity, RigidBody base)
   {
      angularVelocity.changeFrame(endEffectorFrame);
      linearVelocityOfOrigin.changeFrame(endEffectorFrame);

      return new Twist(endEffectorFrame, base.getBodyFixedFrame(), endEffectorFrame, linearVelocityOfOrigin.getVector(), angularVelocity.getVector());
   }

   public SpatialAccelerationVector calculateDesiredEndEffectorSpatialAcceleration(FrameVector linearAccelerationOfOrigin,
           FrameVector angularAcceleration, RigidBody base)
   {
      angularAcceleration.changeFrame(endEffectorFrame);

      linearAccelerationOfOrigin.changeFrame(endEffectorFrame);
      Twist twistOfEndEffectorWithRespectToElevator = new Twist();
      twistCalculator.packRelativeTwist(twistOfEndEffectorWithRespectToElevator, base, endEffector);
      twistOfEndEffectorWithRespectToElevator.changeBodyFrameNoRelativeTwist(endEffectorFrame);
      SpatialAccelerationVector spatialAcceleration = new SpatialAccelerationVector(endEffectorFrame, base.getBodyFixedFrame(), endEffectorFrame);
      spatialAcceleration.setBasedOnOriginAcceleration(angularAcceleration, linearAccelerationOfOrigin, twistOfEndEffectorWithRespectToElevator);

      return spatialAcceleration;
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

   public ReferenceFrame getTrackingFrame()
   {
      return endEffectorFrame;
   }

   public void setGains(SE3PDGains gains)
   {
      se3pdController.getPositionController().setProportionalGains(gains.getPositionProportionalGains());
      se3pdController.getPositionController().setDerivativeGains(gains.getPositionDerivativeGains());

      se3pdController.getOrientationController().setProportionalGains(gains.getOrientationProportionalGains());
      se3pdController.getOrientationController().setDerivativeGains(gains.getOrientationDerivativeGains());
   }
}

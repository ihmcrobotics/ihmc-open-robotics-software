package us.ihmc.robotics.math.trajectories;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;


public class PositionTrajectorySmoother implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   
   private final YoFramePoint yoSmoothedPosition;
   private final YoFrameVector yoSmoothedVelocity;
   private final YoFrameVector yoSmoothedAcceleration;

   private final FramePoint3D smoothedPosition = new FramePoint3D();
   private final FrameVector3D smoothedVelocity = new FrameVector3D();
   private final FrameVector3D smoothedAcceleration = new FrameVector3D();

   private final FramePoint3D positionInput;
   private final FrameVector3D velocityInput;
   private final FrameVector3D accelerationInput;

   private final FrameVector3D positionError;
   private final FrameVector3D velocityError;
   private final FrameVector3D accelerationError;
   
   private final FrameVector3D desiredJerk;
   
   private final FrameVector3D tempVector;
   
   private final YoDouble positionGain, velocityGain, accelerationGain;
   
   private final PositionTrajectoryGenerator positionTrajectoryInput;
   
   private final double dt;

   private double maximumAcceleration = Double.POSITIVE_INFINITY;
   private double maximumJerk = Double.POSITIVE_INFINITY;
   
   private final ReferenceFrame trajectoryReferenceFrame;


   public PositionTrajectorySmoother(String namePrefix, PositionTrajectoryGenerator positionTrajectoryInput, double dt, YoVariableRegistry parentRegistry)
   {
      this.positionTrajectoryInput = positionTrajectoryInput;
      this.dt = dt;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      positionTrajectoryInput.getLinearData(smoothedPosition, smoothedVelocity, smoothedAcceleration);
      trajectoryReferenceFrame = smoothedPosition.getReferenceFrame();

      yoSmoothedPosition = new YoFramePoint(namePrefix + "SmoothedPosition", trajectoryReferenceFrame, registry);
      yoSmoothedVelocity = new YoFrameVector(namePrefix + "SmoothedVelocity", trajectoryReferenceFrame, registry);
      yoSmoothedAcceleration = new YoFrameVector(namePrefix + "SmoothedAcceleration", trajectoryReferenceFrame, registry);

      positionGain = new YoDouble(namePrefix + "PositionGain", registry);
      velocityGain = new YoDouble(namePrefix + "VelocityGain", registry);
      accelerationGain = new YoDouble(namePrefix + "AccelerationGain", registry);
      
      positionInput = new FramePoint3D(trajectoryReferenceFrame);
      velocityInput = new FrameVector3D(trajectoryReferenceFrame);
      accelerationInput = new FrameVector3D(trajectoryReferenceFrame);

      positionError = new FrameVector3D(trajectoryReferenceFrame);
      velocityError = new FrameVector3D(trajectoryReferenceFrame);
      accelerationError = new FrameVector3D(trajectoryReferenceFrame);
      
      desiredJerk = new FrameVector3D(trajectoryReferenceFrame);
      
      tempVector = new FrameVector3D(trajectoryReferenceFrame);
      
      parentRegistry.addChild(registry);
   }
   public void setGains(double positionGain, double velocityGain, double acclerationGain)
   {
      this.positionGain.set(positionGain);
      this.velocityGain.set(velocityGain);
      this.accelerationGain.set(acclerationGain);
   }
   public void computeGainsByPolePlacement(double w0, double w1, double zeta1)
   {
      positionGain.set(w0 * w1 * w1);
      velocityGain.set(w1*w1 + 2.0 * zeta1 * w1 * w0);
      accelerationGain.set(w0 + 2.0 * zeta1 * w1);
   }
   
   public void setMaxAccelerationAndJerk(double maxAbsoluteAcceleration, double maxAbsoluteJerk)
   {
      maximumAcceleration = Math.abs(maxAbsoluteAcceleration);
      maximumJerk = Math.abs(maxAbsoluteJerk);
   }
   
   public void initialize()
   {
      positionTrajectoryInput.initialize();
      setRawData();
   }

   private void setRawData()
   {
      positionTrajectoryInput.getLinearData(smoothedPosition, smoothedVelocity, smoothedAcceleration);
      yoSmoothedPosition.set(smoothedPosition);
      yoSmoothedVelocity.set(smoothedVelocity);
      smoothedAcceleration.setToZero(trajectoryReferenceFrame);
      yoSmoothedAcceleration.set(smoothedAcceleration);
   }

   public void compute(double time)
   {
      positionTrajectoryInput.compute(time);
      
      if (time < 1e-10)
      {
         setRawData();
         return;
      }
      
      positionTrajectoryInput.getLinearData(positionInput, velocityInput, accelerationInput);
      
      positionError.sub(positionInput, smoothedPosition);
      velocityError.sub(velocityInput, smoothedVelocity);
      accelerationError.sub(accelerationInput, smoothedAcceleration);

      positionError.scale(positionGain.getDoubleValue());
      velocityError.scale(velocityGain.getDoubleValue());
      accelerationError.scale(accelerationGain.getDoubleValue());
      
      desiredJerk.set(positionError);
      desiredJerk.add(velocityError);
      desiredJerk.add(accelerationError);
      
      double jerkMagnitude = desiredJerk.length();
      
      if (jerkMagnitude > maximumJerk)
      {
         desiredJerk.scale(maximumJerk / jerkMagnitude);
      }
      
      tempVector.set(desiredJerk);
      tempVector.scale(dt);
      smoothedAcceleration.add(tempVector);
      
      double accelerationMagnitude = smoothedAcceleration.length();
      
      if (accelerationMagnitude > maximumAcceleration)
      {
         smoothedAcceleration.scale(maximumAcceleration / accelerationMagnitude);
      }

      tempVector.set(smoothedAcceleration);
      tempVector.scale(dt);
      smoothedVelocity.add(tempVector);
      
      tempVector.set(smoothedVelocity);
      tempVector.scale(dt);
      smoothedPosition.add(tempVector);
      
      yoSmoothedPosition.set(smoothedPosition);
      yoSmoothedVelocity.set(smoothedVelocity);
      yoSmoothedAcceleration.set(smoothedAcceleration);
   }

   private final double errorThreshold = 1e-3;
   
   public boolean isDone()
   {
      boolean isTrajectoryInputDone = positionTrajectoryInput.isDone();
      boolean isSmoothingDone = positionError.lengthSquared() < errorThreshold && velocityError.lengthSquared() < errorThreshold && accelerationError.lengthSquared() < errorThreshold;
      return isTrajectoryInputDone && isSmoothingDone;
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(yoSmoothedPosition);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(yoSmoothedVelocity);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(yoSmoothedAcceleration);
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void showVisualization()
   {
   }

   public void hideVisualization()
   {
   }
}

package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoSpline3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SoftTouchdownPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry;
   
   private final YoSpline3D positionTouchdownTrajectory;
   private final FramePoint3D desiredPosition = new FramePoint3D();
   
   private final YoDouble timeInitial;
   private final YoDouble timeFinal;
   private final YoDouble timeIntoTouchdown;
   
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FrameVector3D initialAcceleration = new FrameVector3D();
   
   private final FrameQuaternion constantOrientation = new FrameQuaternion();
   private final FrameVector3D constantAngularVelocity = new FrameVector3D();
   private final FrameVector3D constantAngularAcceleration = new FrameVector3D();
   
   public SoftTouchdownPoseTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      
      positionTouchdownTrajectory = new YoSpline3D(3, 3, worldFrame, registry, namePrefix + "Trajectory");
      timeInitial = new YoDouble(namePrefix + "TimeInitial", registry);
      timeFinal = new YoDouble(namePrefix + "TimeFinal", registry);
      timeIntoTouchdown = new YoDouble(namePrefix + "TimeIntoTouchdown", registry);
      
      timeFinal.set(Double.POSITIVE_INFINITY);
   }
   
   public void setOrientation(FrameQuaternionReadOnly orientation)
   {
      constantOrientation.setIncludingFrame(orientation);
      constantAngularVelocity.setToZero(worldFrame);
      constantAngularAcceleration.setToZero(worldFrame);
   }

   public void setOrientation(FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      constantOrientation.setIncludingFrame(orientation);
      constantAngularVelocity.setIncludingFrame(angularVelocity);
      constantAngularAcceleration.setToZero(worldFrame);
   }

   public void setLinearTrajectory(double initialTime, FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity, FrameVector3DReadOnly initialAcceleration)
   {
      this.timeInitial.set(initialTime);
      
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
      this.initialAcceleration.setIncludingFrame(initialAcceleration);
      
      this.initialPosition.changeFrame(worldFrame);
      this.initialVelocity.changeFrame(worldFrame);
      this.initialAcceleration.changeFrame(worldFrame);
   }

   @Override
   public void initialize()
   {
      double t0 = timeInitial.getDoubleValue();
      double tf = timeFinal.getDoubleValue();
      positionTouchdownTrajectory.setQuadraticUsingInitialVelocityAndAcceleration(t0, tf, initialPosition, initialVelocity, initialAcceleration);
   }

   @Override
   public void compute(double time)
   {
      double clippedTime = MathTools.clamp(time, timeInitial.getDoubleValue(), timeFinal.getDoubleValue());
      timeIntoTouchdown.set(clippedTime - timeInitial.getDoubleValue());
      positionTouchdownTrajectory.compute(clippedTime);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionTouchdownTrajectory.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      positionTouchdownTrajectory.getVelocity(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      positionTouchdownTrajectory.getAcceleration(accelerationToPack);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(constantOrientation);
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(constantAngularVelocity);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setIncludingFrame(constantAngularAcceleration);
   }

   @Override
   public void getPose(FramePose3D framePoseToPack)
   {
      getPosition(desiredPosition);
      framePoseToPack.set(desiredPosition, constantOrientation);
   }
   
   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}

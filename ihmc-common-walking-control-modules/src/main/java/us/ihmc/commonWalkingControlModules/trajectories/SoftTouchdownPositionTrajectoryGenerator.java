package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoSpline3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SoftTouchdownPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry;

   private final YoSpline3D positionTouchdownTrajectory;

   private final YoDouble timeInitial;
   private final YoDouble timeFinal;
   private final YoDouble timeIntoTouchdown;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FrameVector3D initialAcceleration = new FrameVector3D();

   public SoftTouchdownPositionTrajectoryGenerator(String namePrefix, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      
      positionTouchdownTrajectory = new YoSpline3D(3, 3, worldFrame, registry, namePrefix + "Trajectory");
      timeInitial = new YoDouble(namePrefix + "TimeInitial", registry);
      timeFinal = new YoDouble(namePrefix + "TimeFinal", registry);
      timeIntoTouchdown = new YoDouble(namePrefix + "TimeIntoTouchdown", registry);
      
      timeFinal.set(Double.POSITIVE_INFINITY);
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
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}

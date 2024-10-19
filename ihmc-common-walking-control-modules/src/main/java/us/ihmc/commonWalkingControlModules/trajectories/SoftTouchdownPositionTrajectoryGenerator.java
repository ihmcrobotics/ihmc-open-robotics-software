package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.commons.trajectories.interfaces.FixedFramePolynomial3DBasics;
import us.ihmc.commons.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.commons.trajectories.yoVariables.YoFramePolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SoftTouchdownPositionTrajectoryGenerator implements FixedFramePositionTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry;

   private final FixedFramePolynomial3DBasics positionTouchdownTrajectory;

   private final YoDouble timeInitial;
   private final YoDouble timeFinal;
   private final YoDouble timeIntoTouchdown;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FrameVector3D initialAcceleration = new FrameVector3D();

   public SoftTouchdownPositionTrajectoryGenerator(String namePrefix, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      
      positionTouchdownTrajectory = new YoFramePolynomial3D( namePrefix + "Trajectory", 3, worldFrame, registry);
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
      positionTouchdownTrajectory.setQuadraticUsingInitialAcceleration(t0, tf, initialPosition, initialVelocity, initialAcceleration);
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
   public FramePoint3DReadOnly getPosition()
   {
      return positionTouchdownTrajectory.getPosition();
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return positionTouchdownTrajectory.getVelocity();
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return positionTouchdownTrajectory.getAcceleration();
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

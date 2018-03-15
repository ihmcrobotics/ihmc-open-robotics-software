package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoSpline3D;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SoftTouchdownPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final String namePostFix = getClass().getSimpleName();

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   
   private final ReferenceFrame referenceFrame;

   private final PositionProvider initialPositionSource;
   private final FrameVector3DReadOnly touchdownVelocity;
   private final FrameVector3DReadOnly touchdownAcceleration;
   
   private final DoubleProvider startTimeProvider;
   
   private final YoDouble startTime;
   private final YoDouble timeIntoTouchdown;
   
   private final FramePoint3D p0;
   private final FrameVector3D pd0;
   private final FrameVector3D pdd0;
   
   private final double tf = Double.POSITIVE_INFINITY;
   
   private final YoBoolean replanningTrajectory;

   private final YoSpline3D trajectory;

   public SoftTouchdownPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider initialPositionProvider,
                                                   FrameVector3DReadOnly touchdownVelocity, FrameVector3DReadOnly touchdownAcceleration, DoubleProvider startTimeProvider,
                                                   YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);
      
      p0 = new FramePoint3D();
      pd0 = new FrameVector3D();
      pdd0 = new FrameVector3D();
      
      this.replanningTrajectory = new YoBoolean(namePrefix + "ReplanningTrajectory", parentRegistry);
      this.replanningTrajectory.set(false);
      
      this.referenceFrame = referenceFrame;

      initialPositionSource = initialPositionProvider;
      this.touchdownVelocity = touchdownVelocity;
      this.touchdownAcceleration = touchdownAcceleration;
      
      this.startTimeProvider = startTimeProvider;
      
      startTime = new YoDouble(namePrefix + "startTime", registry);
      startTime.set(startTimeProvider.getValue());

      timeIntoTouchdown = new YoDouble(namePrefix + "timeIntoTouchdown", registry);

      trajectory = new YoSpline3D(3, 3, referenceFrame, registry, namePrefix + "Trajectory");
   }

   public SoftTouchdownPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);
      
      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);
      
      p0 = new FramePoint3D();
      pd0 = new FrameVector3D();
      pdd0 = new FrameVector3D();
      
      this.replanningTrajectory = new YoBoolean(namePrefix + "ReplanningTrajectory", parentRegistry);
      this.replanningTrajectory.set(false);
      
      this.referenceFrame = referenceFrame;
      
      initialPositionSource = null;
      touchdownVelocity = null;
      touchdownAcceleration = null;
      this.startTimeProvider = null;
      
      startTime = new YoDouble(namePrefix + "startTime", registry);

      timeIntoTouchdown = new YoDouble(namePrefix + "timeIntoTouchdown", registry);
      trajectory = new YoSpline3D(3, 3, referenceFrame, registry, namePrefix + "Trajectory");
   }

   @Override
   public void initialize()
   {
      double t0 = startTimeProvider.getValue();
      startTime.set(startTimeProvider.getValue());
      timeIntoTouchdown.set(0.0);
        
      initialPositionSource.getPosition(p0);
      p0.changeFrame(referenceFrame);
     
      pd0.setIncludingFrame(touchdownVelocity);
      pd0.changeFrame(referenceFrame);

      pdd0.setIncludingFrame(touchdownAcceleration);
      pdd0.changeFrame(referenceFrame);
     
      trajectory.setQuadraticUsingInitialVelocityAndAcceleration(t0, tf, p0, pd0, pdd0);
   }

   public void initialize(double t0, FramePoint3D initialPosition, FrameVector3D initalVelocity, FrameVector3D initialAcceleration)
   {
      startTime.set(t0);
      timeIntoTouchdown.set(0.0);
      
      p0.setIncludingFrame(initialPosition);
      p0.changeFrame(referenceFrame);
      
      pd0.setIncludingFrame(initalVelocity);
      pd0.changeFrame(referenceFrame);
      
      pdd0.setIncludingFrame(initialAcceleration);
      pdd0.changeFrame(referenceFrame);
      
      trajectory.setQuadraticUsingInitialVelocityAndAcceleration(t0, tf, p0, pd0, pdd0);
   }

   @Override
   public void compute(double time)
   {
      timeIntoTouchdown.set(time - startTime.getDoubleValue());
      
      trajectory.compute(time);
      desiredPosition.set(trajectory.getPosition());
      desiredVelocity.set(trajectory.getVelocity());
      desiredAcceleration.set(trajectory.getAcceleration());
   }
   
   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(desiredPosition);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(desiredVelocity);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(desiredAcceleration);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
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

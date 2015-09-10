package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.YoSpline3D;


public class SoftTouchdownTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final String namePostFix = getClass().getSimpleName();

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   
   private final ReferenceFrame referenceFrame;

   private final PositionProvider initialPositionSource;
   private final VectorProvider velocitySource;
   
   private final DoubleProvider startTimeProvider;
   
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable timeIntoTouchdown;
   
   private final FramePoint p0;
   private final FrameVector pd0;
   
   private final double tf = Double.POSITIVE_INFINITY;
   private double t0;
   
   private final BooleanYoVariable replanningTrajectory;

   private final YoSpline3D trajectory;

   public SoftTouchdownTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider initialPositionProvider,
           VectorProvider velocityProvider, DoubleProvider startTimeProvider, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);
      
      p0 = new FramePoint();
      pd0 = new FrameVector();
      t0 = 0.0;
      
      this.replanningTrajectory = new BooleanYoVariable(namePrefix + "ReplanningTrajectory", parentRegistry);
      this.replanningTrajectory.set(false);
      
      this.referenceFrame = referenceFrame;

      initialPositionSource = initialPositionProvider;
      velocitySource = velocityProvider;
      
      this.startTimeProvider = startTimeProvider;
      
      startTime = new DoubleYoVariable(namePrefix + "startTime", registry);

      timeIntoTouchdown = new DoubleYoVariable(namePrefix + "timeIntoTouchdown", registry);

      trajectory = new YoSpline3D(2, 2, referenceFrame, registry, namePrefix + "Trajectory");
   }

   public void initialize()
   {
	   if(!replanningTrajectory.getBooleanValue())
	   {
		   setInitialTimePositionsAndVelocities();
	   }
      
      trajectory.setLinearUsingInitialPositionAndVelocity(t0, tf, p0, pd0);
      replanningTrajectory.set(false);
   }

   public void compute(double time)
   {
      timeIntoTouchdown.set(time - startTime.getDoubleValue());
      
      trajectory.compute(time);
      desiredPosition.set(trajectory.getPosition());
      desiredVelocity.set(trajectory.getVelocity());
      desiredAcceleration.set(trajectory.getAcceleration());
   }
   
   public void setInitialTimePositionsAndVelocities()
   {
	   startTime.set(startTimeProvider.getValue());
	   t0 = startTime.getDoubleValue();
	   timeIntoTouchdown.set(0.0);
	      
	   initialPositionSource.get(p0);
	   p0.changeFrame(referenceFrame);
	   
	   velocitySource.get(pd0);
	   pd0.changeFrame(referenceFrame);
   }

   public boolean isDone()
   {
      return false;
   }
   
   public void setInitialPosition(FramePoint newInitialPosition)
   {
	   if(!replanningTrajectory.getBooleanValue())
	   {
		   throw new RuntimeException("You must set the boolean replanningTrajectory before you are allowed to set a new initial position or velocity");
	   }
	   else
	   {
		   p0.set(newInitialPosition);
	   }
   }
   
   public void setInitialVelocity(FrameVector newInitialVelocity)
   {
	   if(!replanningTrajectory.getBooleanValue())
	   {
		   throw new RuntimeException("You must set the boolean replanningTrajectory before you are allowed to set a new initial position or velocity");
	   }
	   else
	   {
		   pd0.set(newInitialVelocity);
	   }

   }
   
   public void setReplanningTrajectoryBoolean(boolean value)
   {
	   replanningTrajectory.set(true);
   }
   
   public YoFramePoint getDesiredPosition()
   {
      return desiredPosition;
   }
   
   public YoFrameVector getDesiredVelocity()
   {
      return desiredVelocity;
   }
   
   public YoFrameVector getDesiredAcceleration()
   {
      return desiredAcceleration;
   }
}

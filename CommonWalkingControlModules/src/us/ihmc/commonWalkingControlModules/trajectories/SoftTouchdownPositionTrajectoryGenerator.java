package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoSpline3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;


public class SoftTouchdownPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final String namePostFix = getClass().getSimpleName();

   private final YoFramePoint desiredPosition;
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector desiredAcceleration;
   
   private final ReferenceFrame referenceFrame;

   private final PositionProvider initialPositionSource;
   private final VectorProvider velocitySource;
   private final VectorProvider accelerationSource;
   
   private final DoubleProvider startTimeProvider;
   
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable timeIntoTouchdown;
   
   private final FramePoint p0;
   private final FrameVector pd0;
   private final FrameVector pdd0;
   
   private final double tf = Double.POSITIVE_INFINITY;
   private double t0;
   
   private final BooleanYoVariable replanningTrajectory;

   private final YoSpline3D trajectory;

   public SoftTouchdownPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider initialPositionProvider,
           VectorProvider velocityProvider, VectorProvider accelerationProvider, DoubleProvider startTimeProvider, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);
      
      p0 = new FramePoint();
      pd0 = new FrameVector();
      pdd0 = new FrameVector();
      t0 = 0.0;
      
      this.replanningTrajectory = new BooleanYoVariable(namePrefix + "ReplanningTrajectory", parentRegistry);
      this.replanningTrajectory.set(false);
      
      this.referenceFrame = referenceFrame;

      initialPositionSource = initialPositionProvider;
      velocitySource = velocityProvider;
      accelerationSource = accelerationProvider;
      
      this.startTimeProvider = startTimeProvider;
      
      startTime = new DoubleYoVariable(namePrefix + "startTime", registry);
      startTime.set(startTimeProvider.getValue());

      timeIntoTouchdown = new DoubleYoVariable(namePrefix + "timeIntoTouchdown", registry);

      trajectory = new YoSpline3D(3, 3, referenceFrame, registry, namePrefix + "Trajectory");
   }

   @Override
   public void initialize()
   {
	  setInitialTimePositionsAndVelocities();
      
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
   
   public void setInitialTimePositionsAndVelocities()
   {
	   t0 = startTimeProvider.getValue();
	   startTime.set(startTimeProvider.getValue());
	   timeIntoTouchdown.set(0.0);
	      
	   initialPositionSource.getPosition(p0);
	   p0.changeFrame(referenceFrame);
	   
	   velocitySource.get(pd0);
	   pd0.changeFrame(referenceFrame);

	   accelerationSource.get(pdd0);
	   pdd0.changeFrame(referenceFrame);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public void getPosition(FramePoint positionToPack)
   {
      desiredPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
      // TODO Auto-generated method stub
   }

   @Override
   public void hideVisualization()
   {
      // TODO Auto-generated method stub
   }
}

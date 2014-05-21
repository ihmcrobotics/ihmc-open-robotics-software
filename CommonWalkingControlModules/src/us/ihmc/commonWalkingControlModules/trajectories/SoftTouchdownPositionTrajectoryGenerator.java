package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;
import com.yobotics.simulationconstructionset.util.trajectory.DoubleProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionProvider;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import com.yobotics.simulationconstructionset.util.trajectory.VectorProvider;
import com.yobotics.simulationconstructionset.util.trajectory.YoSpline3D;

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
   
   private final DoubleProvider startTimeProvider;
   
   private final DoubleYoVariable startTime;
   private final DoubleYoVariable timeIntoTouchdown;
   
   private final FramePoint p0;
   private final FrameVector pd0;
   
   private final double tf = Double.POSITIVE_INFINITY;
   private double t0;
   
   private final BooleanYoVariable replanningTrajectory;

   private final YoSpline3D trajectory;

   public SoftTouchdownPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider initialPositionProvider,
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
      startTime.set(startTimeProvider.getCurrentSwingTimeValue());

      timeIntoTouchdown = new DoubleYoVariable(namePrefix + "timeIntoTouchdown", registry);

      trajectory = new YoSpline3D(2, 2, referenceFrame, registry, namePrefix + "Trajectory");
   }

   public void initialize()
   {
	  setInitialTimePositionsAndVelocities();
      
      trajectory.setLinearUsingInitialPositionAndVelocity(t0, tf, p0, pd0);
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

   public void get(FramePoint positionToPack)
   {
      desiredPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void packVelocity(FrameVector velocityToPack)
   {
      desiredVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void packAcceleration(FrameVector accelerationToPack)
   {
      desiredAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }
}

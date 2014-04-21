package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
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

   private final YoSpline3D trajectory;

   public SoftTouchdownPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, PositionProvider initialPositionProvider,
           VectorProvider velocityProvider, DoubleProvider startTimeProvider, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

      desiredPosition = new YoFramePoint(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector(namePrefix + "DesiredAcceleration", referenceFrame, registry);
      
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
      startTime.set(startTimeProvider.getValue());
      timeIntoTouchdown.set(0.0);
      
      double t0 = startTime.getDoubleValue();
      double tf = Double.POSITIVE_INFINITY;
      
      FramePoint p0 = new FramePoint();
      initialPositionSource.get(p0);
      p0.changeFrame(referenceFrame);
      
      FrameVector pd0 = new FrameVector();
      velocitySource.get(pd0);
      pd0.changeFrame(referenceFrame);
      
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

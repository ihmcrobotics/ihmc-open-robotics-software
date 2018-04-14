package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class ThreeDoFSwingFootTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoPolynomial3D swingTrajectory;

   private final YoDouble duration;
   private final YoDouble timeInState;

   private final YoDouble groundClearance;

   private final YoBoolean isDone;

   final private FramePoint3D initialPosition = new FramePoint3D();
   final private FrameVector3D initialVelocity = new FrameVector3D();

   final private FramePoint3D finalPosition = new FramePoint3D();
   final private FrameVector3D finalVelocity = new FrameVector3D();

   private ReferenceFrame referenceFrame;
   private boolean initialized;

   private final YoGraphicPolynomial3D trajectoryViz;

   private final String prefix;
   private final boolean debug;

   public ThreeDoFSwingFootTrajectoryGenerator(String prefix, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this(prefix, registry, graphicsListRegistry, false);
   }

   public ThreeDoFSwingFootTrajectoryGenerator(String prefix, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry, boolean debug)
   {
      this.debug = debug;
      this.prefix = prefix;
      swingTrajectory = new YoPolynomial3D(prefix + "SwingTrajectory", 4, registry);
      duration = new YoDouble(prefix + "SwingDuration", registry);
      timeInState = new YoDouble(prefix + "TimeInSwingState", registry);
      groundClearance = new YoDouble(prefix + "SwingGroundClearance", registry);
      isDone = new YoBoolean(prefix + "SwingIsDone", registry);

      referenceFrame = ReferenceFrame.getWorldFrame();
      initialized = false;

      if (graphicsListRegistry != null)
      {
         trajectoryViz = new YoGraphicPolynomial3D(prefix + "Trajectory", swingTrajectory, duration, 0.01, 50, 8, registry);
         graphicsListRegistry.registerYoGraphic(prefix + "Trajectory", trajectoryViz);

         trajectoryViz.setColorType(YoGraphicPolynomial3D.TrajectoryColorType.ACCELERATION_BASED);
      }
      else
         trajectoryViz = null;
   }

   private void visualize()
   {
      if (trajectoryViz == null)
         return;

      trajectoryViz.showGraphic();
   }

   @Override
   public void getPosition(FramePoint3D position)
   {
      position.setIncludingFrame(referenceFrame, swingTrajectory.getPosition());
   }

   @Override
   public void getVelocity(FrameVector3D velocity)
   {
      velocity.setIncludingFrame(referenceFrame, swingTrajectory.getVelocity());
      velocity.scale(1.0 / duration.getDoubleValue());
   }

   @Override
   public void getAcceleration(FrameVector3D acceleration)
   {
      acceleration.setIncludingFrame(referenceFrame, swingTrajectory.getAcceleration());
      acceleration.scale(1.0 / (duration.getDoubleValue() * duration.getDoubleValue()));
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void setStepDuration(double duration)
   {
      this.duration.set(duration);
   }

   public void setGroundClearance(double groundClearance)
   {
      this.groundClearance.set(groundClearance);
   }

   @Override
   public void showVisualization()
   {
      trajectoryViz.showGraphic();
   }

   @Override
   public void hideVisualization()
   {
      trajectoryViz.hideGraphic();
   }

   public void setInitialConditions(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity)
   {
      referenceFrame = initialPosition.getReferenceFrame();

      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialVelocity.setIncludingFrame(initialVelocity);
      this.initialVelocity.changeFrame(referenceFrame);

   }

   public void setFinalConditions(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalVelocity.setIncludingFrame(finalVelocity);
      this.finalPosition.changeFrame(referenceFrame);
      this.finalVelocity.changeFrame(referenceFrame);

   }

   public void initialize()
   {
      timeInState.set(0.0);
      isDone.set(false);

      if (debug)
      {
         if (initialPosition.containsNaN())
            throw new RuntimeException("Initial position for " + prefix + " is invalid.");
         if (initialVelocity.containsNaN())
            throw new RuntimeException("Initial velocity for " + prefix + " is invalid.");
         if (finalPosition.containsNaN())
            throw new RuntimeException("Final position for " + prefix + " is invalid.");
         if (finalVelocity.containsNaN())
            throw new RuntimeException("Final velocity for " + prefix + " is invalid.");
      }

      double midwayPositionZ = groundClearance.getDoubleValue() + Math.max(initialPosition.getZ(), finalPosition.getZ());

      swingTrajectory.getYoPolynomialX().setCubic(0, 1.0, initialPosition.getX(), initialVelocity.getX(), finalPosition.getX(), finalVelocity.getX());
      swingTrajectory.getYoPolynomialY().setCubic(0, 1.0, initialPosition.getY(), initialVelocity.getY(), finalPosition.getY(), finalVelocity.getY());
      swingTrajectory.getYoPolynomialZ().setQuadraticUsingIntermediatePoint(0, 0.5, 1.0, initialPosition.getZ(), midwayPositionZ, finalPosition.getZ());

      initialized = true;
      compute(0.0);
      visualize();
   }

   @Override
   public void compute(double timeInState)
   {
      if (!initialized)
      {
         throw new RuntimeException("parameters must be initialized before computing trajectory");
      }
      if (debug && !Double.isFinite(timeInState))
      {
         throw new RuntimeException("Time in state for " + prefix + " is invalid.");
      }

      double trajectoryTime = duration.getDoubleValue();
      isDone.set(timeInState >= trajectoryTime);

      timeInState = MathTools.clamp(timeInState, 0.0, trajectoryTime);

      double percent = timeInState / trajectoryTime;
      swingTrajectory.compute(percent);
   }

   public boolean isDone()
   {
      return timeInState.getDoubleValue() >= duration.getDoubleValue();
   }
}
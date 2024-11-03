package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.NDoFTrapezoidalVelocityTrajectory.AlphaToAlphaType;

/**
 * WARNING this is super not realtime safe and makes a ton of garbage.
 */
@Deprecated
public class StraightLineCartesianTrajectoryGenerator implements FixedFramePositionTrajectoryGenerator
{
   private final YoRegistry registry;
   private final ReferenceFrame referenceFrame;
   private final YoDouble time;
   private final YoDouble maxVel;
   private final YoDouble maxAccel;

   private FramePointTrapezoidalVelocityTrajectory trajectory;

   private final FixedFramePoint3DBasics initialPosition;
   private final FixedFrameVector3DBasics initialVelocity;
   private final FixedFrameVector3DBasics initialAcceleration;
   private final FixedFramePoint3DBasics finalDesiredPosition;
   private final FixedFrameVector3DBasics finalDesiredVelocity;

   private final FramePoint3D desiredPosition;
   private final FrameVector3D desiredVelocity;
   private final FrameVector3D desiredAcceleration;

   public StraightLineCartesianTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, double maxVel, double maxAccel, YoDouble time, YoRegistry parentRegistry)
   {
      this.registry = new YoRegistry(namePrefix + "StraightLineCartesianTrajectoryGenerator");
      this.maxVel = new YoDouble("straightLineTrajMaxVel", registry);
      this.maxAccel = new YoDouble("straightLineTrajMaxAccel", registry);

      this.referenceFrame = referenceFrame;
      this.time = time;

      initialPosition = new FramePoint3D(referenceFrame);
      initialVelocity = new FrameVector3D(referenceFrame);
      initialAcceleration = new FrameVector3D(referenceFrame);
      finalDesiredPosition = new FramePoint3D(referenceFrame);
      finalDesiredVelocity = new FrameVector3D(referenceFrame);

      desiredPosition = new FramePoint3D(referenceFrame);
      desiredVelocity = new FrameVector3D(referenceFrame);
      desiredAcceleration = new FrameVector3D(referenceFrame);

      this.maxVel.set(maxVel);
      this.maxAccel.set(maxAccel);
      parentRegistry.addChild(registry);
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void setInitialPosition(FramePoint3DReadOnly initialPosition)
   {
      this.initialPosition.setMatchingFrame(initialPosition);
   }

   public void setInitialVelocity(FrameVector3DReadOnly initialVelocity)
   {
      this.initialVelocity.setMatchingFrame(initialVelocity);
   }

   public void setInitialAcceleration(FrameVector3DReadOnly initialAcceleration)
   {
      this.initialAcceleration.setMatchingFrame(initialAcceleration);
   }

   public void setFinalDesiredPosition(FramePoint3DReadOnly finalDesiredPosition)
   {
      this.finalDesiredPosition.setMatchingFrame(finalDesiredPosition);
   }

   public void setFinalDesiredVelocity(FrameVector3DReadOnly finalDesiredVelocity)
   {
      this.finalDesiredVelocity.setMatchingFrame(finalDesiredVelocity);
   }

   @Override
   public void initialize()
   {
      double maxVelDouble = maxVel.getDoubleValue();
      double maxAccelDouble = maxAccel.getDoubleValue();

      FrameVector3D vMax = new FrameVector3D(referenceFrame, maxVelDouble, maxVelDouble, maxVelDouble);
      FrameVector3D aMax = new FrameVector3D(referenceFrame, maxAccelDouble, maxAccelDouble, maxAccelDouble);

      trajectory = new FramePointTrapezoidalVelocityTrajectory(time.getDoubleValue(),
                                                               initialPosition,
                                                               finalDesiredPosition,
                                                               initialVelocity,
                                                               finalDesiredVelocity,
                                                               vMax,
                                                               aMax,
                                                               AlphaToAlphaType.LINEAR);
   }

   @Override
   public void compute(double time)
   {
      desiredPosition.setMatchingFrame(trajectory.getPosition(time));
      desiredVelocity.setMatchingFrame(trajectory.getVelocity(time));
      desiredAcceleration.setMatchingFrame(trajectory.getAcceleration(time));
   }

   @Override
   public boolean isDone()
   {
      if (trajectory == null)
         return true;

      return (time.getDoubleValue() > trajectory.getTFMax());
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return desiredPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return desiredAcceleration;
   }

   public double getFinalTime()
   {
      return trajectory.getTFMax();
   }

   public void setMaxVelocity(double velocity)
   {
      maxVel.set(velocity);
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

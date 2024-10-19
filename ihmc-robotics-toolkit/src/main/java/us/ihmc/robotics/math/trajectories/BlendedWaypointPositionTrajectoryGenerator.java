package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.commons.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;

public class BlendedWaypointPositionTrajectoryGenerator implements FixedFramePositionTrajectoryGenerator
{
   private final FixedFramePositionTrajectoryGenerator trajectory;
   private final ReferenceFrame trajectoryFrame;

   private final Point3D initialConstraintPositionError = new Point3D();
   private final Vector3D initialConstraintVelocityError = new Vector3D();

   private final Point3D finalConstraintPositionError = new Point3D();
   private final Vector3D finalConstraintVelocityError = new Vector3D();

   private final Point3D constraintPositionError = new Point3D();
   private final Vector3D constraintVelocityError = new Vector3D();

   private static final Point3DReadOnly zeroPoint = EuclidCoreTools.origin3D;
   private static final Vector3DReadOnly zeroVector = EuclidCoreTools.zeroVector3D;

   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D velocity = new FrameVector3D();
   private final FrameVector3D acceleration = new FrameVector3D();
   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempVelocity = new FrameVector3D();

   private final MultipleWaypointsPositionTrajectoryGenerator blendTrajectory;

   public BlendedWaypointPositionTrajectoryGenerator(String prefix,
                                                     FixedFramePositionTrajectoryGenerator trajectory,
                                                     ReferenceFrame trajectoryFrame,
                                                     YoRegistry parentRegistry)
   {
      this.trajectory = trajectory;
      this.trajectoryFrame = trajectoryFrame;

      blendTrajectory = new MultipleWaypointsPositionTrajectoryGenerator(prefix + "BlendTrajectory", 4, trajectoryFrame, parentRegistry);

      this.position.changeFrame(trajectoryFrame);
      this.velocity.changeFrame(trajectoryFrame);
      this.acceleration.changeFrame(trajectoryFrame);
      this.tempPosition.changeFrame(trajectoryFrame);
      this.tempVelocity.changeFrame(trajectoryFrame);

      clear();
   }

   public void clear()
   {
      clearInitialConstraint();
      clearFinalConstraint();
      blendTrajectory.clear();
   }

   public void clearInitialConstraint()
   {
      initialConstraintPositionError.setToZero();
      initialConstraintVelocityError.setToZero();
   }

   public MultipleWaypointsPositionTrajectoryGenerator getBlendTrajectory()
   {
      return blendTrajectory;
   }

   public void clearFinalConstraint()
   {
      finalConstraintPositionError.setToZero();
      finalConstraintVelocityError.setToZero();
   }

   public void blendInitialConstraint(FramePoint3DReadOnly initialPosition, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintStartingError(initialPosition, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendInitialConstraint(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity, double initialTime, double blendDuration)
   {
      clearInitialConstraint();
      computeInitialConstraintStartingError(initialPosition, initialVelocity, initialTime);
      computeInitialConstraintPolynomial(initialTime, blendDuration);
   }

   public void blendFinalConstraint(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintEndingError(finalPosition, finalVelocity, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   public void blendFinalConstraint(FramePoint3DReadOnly finalPosition, double finalTime, double blendDuration)
   {
      clearFinalConstraint();
      computeFinalConstraintEndingError(finalPosition, finalTime);
      computeFinalConstraintPolynomial(finalTime, blendDuration);
   }

   public void addBlendWaypoint(FramePoint3DReadOnly desiredPosition, double time)
   {
      computeConstraintPositionError(desiredPosition, time, constraintPositionError);
      blendTrajectory.appendWaypoint(time, constraintPositionError, zeroVector);
   }

   public void addBlendWaypoint(FramePoint3DReadOnly desiredPosition, FrameVector3DReadOnly desiredVelocity, double time)
   {
      computeConstraintPositionError(desiredPosition, time, constraintPositionError);
      computeConstraintVelocityError(desiredVelocity, constraintVelocityError);
      blendTrajectory.appendWaypoint(time, constraintPositionError, constraintVelocityError);
   }

   public void initializeTrajectory()
   {
      trajectory.initialize();
   }

   public void initializeBlendingTrajectory()
   {
      blendTrajectory.initialize();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return trajectoryFrame;
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return velocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return acceleration;
   }

   @Override
   public void showVisualization()
   {
      trajectory.showVisualization();
   }

   @Override
   public void hideVisualization()
   {
      trajectory.hideVisualization();
   }

   @Override
   public void initialize()
   {
      trajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      trajectory.compute(time);
      position.setIncludingFrame(trajectory.getPosition());
      velocity.setIncludingFrame(trajectory.getVelocity());
      acceleration.setIncludingFrame(trajectory.getAcceleration());

      position.changeFrame(trajectoryFrame);
      velocity.changeFrame(trajectoryFrame);
      acceleration.changeFrame(trajectoryFrame);

      if (!blendTrajectory.isEmpty())
      {
         blendTrajectory.compute(time);
         position.add(blendTrajectory.getPosition());
         velocity.add(blendTrajectory.getVelocity());
         acceleration.add(blendTrajectory.getAcceleration());
      }
   }

   @Override
   public boolean isDone()
   {
      return trajectory.isDone();
   }

   private void computeInitialConstraintStartingError(FramePoint3DReadOnly initialPosition, double initialTime)
   {
      computeConstraintPositionError(initialPosition, initialTime, initialConstraintPositionError);
   }

   private void computeInitialConstraintStartingError(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity, double initialTime)
   {
      computeInitialConstraintStartingError(initialPosition, initialTime);
      computeConstraintVelocityError(initialVelocity, initialConstraintVelocityError);
   }

   private void computeFinalConstraintEndingError(FramePoint3DReadOnly finalPosition, double finalTime)
   {
      computeConstraintPositionError(finalPosition, finalTime, finalConstraintPositionError);
   }

   private void computeFinalConstraintEndingError(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, double finalTime)
   {
      computeFinalConstraintEndingError(finalPosition, finalTime);
      computeConstraintVelocityError(finalVelocity, finalConstraintVelocityError);
   }

   private void computeConstraintPositionError(FramePoint3DReadOnly desiredPosition, double time, Point3DBasics positionErrorToPack)
   {
      trajectory.compute(time);
      trajectoryFrame.checkReferenceFrameMatch(desiredPosition.getReferenceFrame());

      tempPosition.setIncludingFrame(trajectory.getPosition());
      tempPosition.changeFrame(trajectoryFrame);
      positionErrorToPack.sub(desiredPosition, tempPosition);
   }

   private void computeConstraintVelocityError(FrameVector3DReadOnly desiredVelocity, Vector3DBasics velocityErrorToPack)
   {
      trajectoryFrame.checkReferenceFrameMatch(desiredVelocity.getReferenceFrame());

      tempVelocity.setIncludingFrame(trajectory.getVelocity());
      tempVelocity.changeFrame(trajectoryFrame);
      velocityErrorToPack.sub(desiredVelocity, tempVelocity);
   }

   private void computeInitialConstraintPolynomial(double initialTime, double blendDuration)
   {
      blendTrajectory.appendWaypoint(initialTime, initialConstraintPositionError, initialConstraintVelocityError);
      blendTrajectory.appendWaypoint(initialTime + blendDuration, zeroPoint, zeroVector);
      blendTrajectory.initialize();
   }

   private void computeFinalConstraintPolynomial(double finalTime, double blendDuration)
   {
      blendTrajectory.appendWaypoint(finalTime - blendDuration, zeroPoint, zeroVector);
      blendTrajectory.appendWaypoint(finalTime, finalConstraintPositionError, finalConstraintVelocityError);
      blendTrajectory.initialize();
   }
}

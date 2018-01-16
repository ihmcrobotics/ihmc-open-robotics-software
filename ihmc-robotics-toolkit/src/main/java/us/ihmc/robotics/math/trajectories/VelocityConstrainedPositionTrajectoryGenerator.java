package us.ihmc.robotics.math.trajectories;

import org.apache.commons.math3.util.Precision;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class VelocityConstrainedPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;
   private final YoPolynomial xPolynomial, yPolynomial, zPolynomial;

   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;

   private final YoFramePoint initialPosition;
   private final YoFrameVector initialVelocity;

   private final YoFramePoint finalPosition;
   private final YoFrameVector finalVelocity;

   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;

   public VelocityConstrainedPositionTrajectoryGenerator(String name, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(name, false, referenceFrame, parentRegistry);
   }

   public VelocityConstrainedPositionTrajectoryGenerator(String name, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);
      registry = new YoVariableRegistry(name);

      String initialPositionName = name + "InitialPosition";
      String initialVelocityName = name + "InitialVelocity";
      String finalPositionName = name + "FinalPosition";
      String finalVelocityName = name + "FinalVelocity";
      String currentPositionName = name + "CurrentPosition";
      String currentVelocityName = name + "CurrentVelocity";
      String currentAccelerationName = name + "CurrentAcceleration";

      if (allowMultipleFrames)
      {
         YoFramePointInMultipleFrames initialPosition = new YoFramePointInMultipleFrames(initialPositionName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames initialVelocity = new YoFrameVectorInMultipleFrames(initialVelocityName, registry, referenceFrame);

         YoFramePointInMultipleFrames finalPosition = new YoFramePointInMultipleFrames(finalPositionName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames finalVelocity = new YoFrameVectorInMultipleFrames(finalVelocityName, registry, referenceFrame);

         YoFramePointInMultipleFrames currentPosition = new YoFramePointInMultipleFrames(currentPositionName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentVelocity = new YoFrameVectorInMultipleFrames(currentVelocityName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentAcceleration = new YoFrameVectorInMultipleFrames(currentAccelerationName, registry, referenceFrame);

         registerMultipleFramesHolders(initialPosition, initialVelocity, finalPosition, finalVelocity, currentPosition, currentVelocity, currentAcceleration);

         this.initialPosition = initialPosition;
         this.initialVelocity = initialVelocity;
         this.finalPosition = finalPosition;
         this.finalVelocity = finalVelocity;
         this.currentPosition = currentPosition;
         this.currentVelocity = currentVelocity;
         this.currentAcceleration = currentAcceleration;
      }
      else
      {
         initialPosition = new YoFramePoint(initialPositionName, referenceFrame, registry);
         initialVelocity = new YoFrameVector(initialVelocityName, referenceFrame, registry);

         finalPosition = new YoFramePoint(finalPositionName, referenceFrame, registry);
         finalVelocity = new YoFrameVector(finalVelocityName, referenceFrame, registry);

         currentPosition = new YoFramePoint(currentPositionName, referenceFrame, registry);
         currentVelocity = new YoFrameVector(currentVelocityName, referenceFrame, registry);
         currentAcceleration = new YoFrameVector(currentAccelerationName, referenceFrame, registry);
      }

      currentTime = new YoDouble(name + "CurrentTime", registry);
      trajectoryTime = new YoDouble(name + "TrajectoryTime", registry);

      xPolynomial = new YoPolynomial(name + "PolynomialX", 4, registry);
      yPolynomial = new YoPolynomial(name + "PolynomialY", 4, registry);
      zPolynomial = new YoPolynomial(name + "PolynomialZ", 4, registry);

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double duration)
   {
      trajectoryTime.set(duration);
   }

   public void setInitialConditions(FramePoint3D initialPosition, FrameVector3D initialVelocity)
   {
      this.initialPosition.setAndMatchFrame(initialPosition);
      this.initialVelocity.setAndMatchFrame(initialVelocity);
   }

   public void setInitialConditions(YoFramePoint initialPosition, YoFrameVector initialVelocity)
   {
      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);
   }

   public void setInitialConditions(YoFramePoint initialPosition, FrameVector3D initialVelocity)
   {
      this.initialPosition.set(initialPosition);
      this.initialVelocity.setAndMatchFrame(initialVelocity);
   }

   public void setFinalConditions(FramePoint3D finalPosition, FrameVector3D finalVelocity)
   {
      this.finalPosition.setAndMatchFrame(finalPosition);
      this.finalVelocity.setAndMatchFrame(finalVelocity);
   }

   public void setFinalConditions(YoFramePoint finalPosition, YoFrameVector finalVelocity)
   {
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   public void setFinalConditions(YoFramePoint finalPosition, FrameVector3D finalVelocity)
   {
      this.finalPosition.set(finalPosition);
      this.finalVelocity.setAndMatchFrame(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, FramePoint3D initialPosition, FrameVector3D initialVelocity, FramePoint3D finalPosition, FrameVector3D finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);

      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, Point3D initialPosition, Vector3D initialVelocity, Point3D finalPosition, Vector3D finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);

      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempLinearVelocity = new FrameVector3D();

   public void setTrajectoryParameters(FrameEuclideanTrajectoryPoint initialFrameEuclideanWaypoint, FrameEuclideanTrajectoryPoint finalFrameEuclideanWaypoint)
   {
      setTrajectoryTime(finalFrameEuclideanWaypoint.getTime() - initialFrameEuclideanWaypoint.getTime());

      initialFrameEuclideanWaypoint.getPositionIncludingFrame(tempPosition);
      initialFrameEuclideanWaypoint.getLinearVelocityIncludingFrame(tempLinearVelocity);
      initialPosition.set(tempPosition);
      initialVelocity.set(tempLinearVelocity);

      finalFrameEuclideanWaypoint.getPositionIncludingFrame(tempPosition);
      finalFrameEuclideanWaypoint.getLinearVelocityIncludingFrame(tempLinearVelocity);
      finalPosition.set(tempPosition);
      finalVelocity.set(tempLinearVelocity);
   }

   public void setTrajectoryParameters(YoFrameEuclideanTrajectoryPoint initialYoFrameEuclideanWaypoint, YoFrameEuclideanTrajectoryPoint finalYoFrameEuclideanWaypoint)
   {
      setTrajectoryTime(finalYoFrameEuclideanWaypoint.getTime() - initialYoFrameEuclideanWaypoint.getTime());

      initialPosition.set(initialYoFrameEuclideanWaypoint.getPosition());
      initialVelocity.set(initialYoFrameEuclideanWaypoint.getLinearVelocity());

      finalPosition.set(finalYoFrameEuclideanWaypoint.getPosition());
      finalVelocity.set(finalYoFrameEuclideanWaypoint.getLinearVelocity());
   }

   @Override
   public void initialize()
   {
      initializePolynomials();
      compute(0.0);
   }

   private void initializePolynomials()
   {
      xPolynomial.setInitialPositionVelocityZeroFinalHighOrderDerivatives(0.0, trajectoryTime.getDoubleValue(), initialPosition.getX(), initialVelocity.getX(), finalPosition.getX(), finalVelocity.getX());
      yPolynomial.setInitialPositionVelocityZeroFinalHighOrderDerivatives(0.0, trajectoryTime.getDoubleValue(), initialPosition.getY(), initialVelocity.getY(), finalPosition.getY(), finalVelocity.getY());
      zPolynomial.setInitialPositionVelocityZeroFinalHighOrderDerivatives(0.0, trajectoryTime.getDoubleValue(), initialPosition.getZ(), initialVelocity.getZ(), finalPosition.getZ(), finalVelocity.getZ());
   }

   @Override
   public void changeFrame(ReferenceFrame referenceFrame)
   {
      super.changeFrame(referenceFrame);
      initializePolynomials();
   }

   @Override
   public void compute(double time)
   {
      if (Double.isNaN(time))
      {
         throw new RuntimeException("Can not call compute on trajectory generator with time NaN.");
      }

      this.currentTime.set(time);

      if (time < 0.0)
      {
         currentPosition.set(initialPosition);
         currentVelocity.setToZero();
         currentAcceleration.setToZero();
         return;
      }
      if (time > trajectoryTime.getDoubleValue())
      {
         currentPosition.set(finalPosition);
         currentVelocity.setToZero();
         currentAcceleration.setToZero();
         return;
      }

      if (Precision.equals(0.0, trajectoryTime.getDoubleValue()))
      {
         currentPosition.set(initialPosition);
         currentVelocity.set(initialVelocity);
         currentAcceleration.setToZero();
         return;
      }

      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      xPolynomial.compute(time);
      yPolynomial.compute(time);
      zPolynomial.compute(time);

      currentPosition.set(xPolynomial.getPosition(), yPolynomial.getPosition(), zPolynomial.getPosition());
      currentVelocity.set(xPolynomial.getVelocity(), yPolynomial.getVelocity(), zPolynomial.getVelocity());
      currentAcceleration.set(xPolynomial.getAcceleration(), yPolynomial.getAcceleration(), zPolynomial.getAcceleration());
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public double getCurrentTime()
   {
      return this.currentTime.getDoubleValue();
   }

   public void setToDone()
   {
      currentTime.set(trajectoryTime.getDoubleValue() + 0.01);
   }

   public double getTrajectoryTime()
   {
      return trajectoryTime.getDoubleValue();
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void get(YoFramePoint positionToPack)
   {
      positionToPack.set(currentPosition);
   }

   public void getProjectedOntoXYPlane(YoFramePoint2d positionToPack)
   {
      positionToPack.set(currentPosition.getX(), currentPosition.getY());
   }

   public void get(Point3D positionToPack)
   {
      currentPosition.get(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(currentVelocity);
   }

   public void getVelocity(Vector3D velocityToPack)
   {
      currentVelocity.get(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
   {
      accelerationToPack.set(currentAcceleration);
   }

   public void getAcceleration(Vector3D accelerationToPack)
   {
      currentAcceleration.get(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void getLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
   {
      positionToPack.set(currentPosition);
      velocityToPack.set(currentVelocity);
      accelerationToPack.set(currentAcceleration);
   }

   public void getFinalPosition(FramePoint3D finalPosition)
   {
      finalPosition.set(this.finalPosition);
   }

   public YoFramePoint getFinalPosition()
   {
      return finalPosition;
   }

   public YoFramePoint getInitialPosition()
   {
      return initialPosition;
   }

   public YoFrameVector getInitialVelocity()
   {
      return initialVelocity;
   }

   public YoFrameVector getFinalVelocity()
   {
      return finalVelocity;
   }

   public YoPolynomial getXPolynomial()
   {
      return xPolynomial;
   }

   public YoPolynomial getYPolynomial()
   {
      return yPolynomial;
   }

   public YoPolynomial getZPolynomial()
   {
      return zPolynomial;
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

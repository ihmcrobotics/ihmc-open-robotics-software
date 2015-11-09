package us.ihmc.robotics.math.trajectories;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final YoPolynomial xPolynomial, yPolynomial, zPolynomial;

   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   
   private final YoFramePoint initialPosition;
   private final YoFramePoint intermediatePosition;

   private final YoFramePoint finalPosition;
   private final YoFrameVector finalVelocity;

   private final YoFramePoint currentPosition;
   private final YoFrameVector currentVelocity;
   private final YoFrameVector currentAcceleration;

   public ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator(String name, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      
      initialPosition = new YoFramePoint(name + "InitialPosition", referenceFrame, registry);
      intermediatePosition = new YoFramePoint(name + "IntermediatePosition", referenceFrame, registry);
      
      finalPosition = new YoFramePoint(name + "FinalPosition", referenceFrame, registry);
      finalVelocity = new YoFrameVector(name + "FinalVelocity", referenceFrame, registry);

      currentPosition = new YoFramePoint(name + "CurrentPosition", referenceFrame, registry);
      currentVelocity = new YoFrameVector(name + "CurrentVelocity", referenceFrame, registry);
      currentAcceleration = new YoFrameVector(name + "CurrentAcceleration", referenceFrame, registry);

      currentTime = new DoubleYoVariable(name + "CurrentTime", registry);
      trajectoryTime = new DoubleYoVariable(name + "TrajectoryTime", registry);

      xPolynomial = new YoPolynomial(name + "PolynomialX", 3, registry);
      yPolynomial = new YoPolynomial(name + "PolynomialY", 3, registry);
      zPolynomial = new YoPolynomial(name + "PolynomialZ", 4, registry);

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double duration)
   {
      trajectoryTime.set(duration);
   }
   
   public void setInitialConditions(FramePoint initialPosition)
   {
      this.initialPosition.setAndMatchFrame(initialPosition);
   }

   public void setInitialConditions(YoFramePoint initialPosition)
   {
      this.initialPosition.set(initialPosition);
   }

   public void setIntermediateConditions(FramePoint intermediatePosition)
   {
      this.intermediatePosition.set(intermediatePosition);  
   }
   
   public void setIntermediateConditions(YoFramePoint intermediatePosition)
   {
      this.intermediatePosition.set(intermediatePosition);  
   }
   
   public void setFinalConditions(FramePoint finalPosition, FrameVector finalVelocity)
   {
      this.finalPosition.setAndMatchFrame(finalPosition);
      this.finalVelocity.setAndMatchFrame(finalVelocity);
   }

   public void setFinalConditions(YoFramePoint finalPosition, YoFrameVector finalVelocity)
   {
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }
   
   public void setFinalConditions(YoFramePoint finalPosition, FrameVector finalVelocity)
   {
      this.finalPosition.set(finalPosition);
      this.finalVelocity.setAndMatchFrame(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, FramePoint initialPosition, FramePoint intermediatePosition, FramePoint finalPosition, FrameVector finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.intermediatePosition.set(intermediatePosition);
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, YoFramePoint initialPosition, YoFramePoint intermediatePosition, YoFramePoint finalPosition, YoFrameVector finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.intermediatePosition.set(intermediatePosition);
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, Point3d initialPosition, Point3d intermediatePosition, Point3d finalPosition, Vector3d finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.intermediatePosition.set(intermediatePosition);
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   public void initialize()
   {
      currentTime.set(0.0);
      double tIntermediate = trajectoryTime.getDoubleValue() / 2.0;
      xPolynomial.setCubicWithFinalVelocityConstraint(0.0,trajectoryTime.getDoubleValue(), initialPosition.getX(), finalPosition.getX(), finalVelocity.getX());
      yPolynomial.setCubicWithFinalVelocityConstraint(0.0,trajectoryTime.getDoubleValue(), initialPosition.getY(), finalPosition.getY(), finalVelocity.getY());
      zPolynomial.setCubicWithIntermediatePositionAndFinalVelocityConstraint(0.0,  tIntermediate, trajectoryTime.getDoubleValue(), initialPosition.getZ(), intermediatePosition.getZ(), finalPosition.getZ(), finalVelocity.getZ());

      currentPosition.set(initialPosition);
      currentAcceleration.setToZero();
   }

   public void compute(double time)
   {
      this.currentTime.set(time);
      time = MathTools.clipToMinMax(time, 0.0, trajectoryTime.getDoubleValue());
      xPolynomial.compute(time);
      yPolynomial.compute(time);
      zPolynomial.compute(time);
      
      if (time < trajectoryTime.getDoubleValue())
      {
         currentPosition.set(xPolynomial.getPosition(), yPolynomial.getPosition(), zPolynomial.getPosition());
         currentVelocity.set(xPolynomial.getVelocity(), yPolynomial.getVelocity(), zPolynomial.getVelocity());
         currentAcceleration.set(xPolynomial.getAcceleration(), yPolynomial.getAcceleration(), zPolynomial.getAcceleration());
      }
      else
      {
         currentPosition.set(finalPosition);
         currentVelocity.set(finalVelocity);
         currentAcceleration.set(0.0, 0.0, 0.0);
      }
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void get(FramePoint positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   public void get(YoFramePoint positionToPack)
   {
      positionToPack.set(currentPosition);
   }

   public void get(Point3d positionToPack)
   {
      currentPosition.get(positionToPack);
   }

   @Override
   public void packVelocity(FrameVector velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void packVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(currentVelocity);
   }

   public void packVelocity(Vector3d velocityToPack)
   {
      currentVelocity.get(velocityToPack);
   }

   @Override
   public void packAcceleration(FrameVector accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void packAcceleration(Vector3d accelerationToPack)
   {
      currentAcceleration.get(accelerationToPack);
   }

   @Override
   public void packLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      get(positionToPack);
      packVelocity(velocityToPack);
      packAcceleration(accelerationToPack);
   }

   public void packLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
   {
      positionToPack.set(currentPosition);
      velocityToPack.set(currentVelocity);
      accelerationToPack.set(currentAcceleration);
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

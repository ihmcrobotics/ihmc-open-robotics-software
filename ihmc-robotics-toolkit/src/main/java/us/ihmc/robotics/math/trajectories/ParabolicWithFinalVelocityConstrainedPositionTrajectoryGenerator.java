package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoRegistry registry;
   private final YoPolynomial xPolynomial, yPolynomial, zPolynomial;

   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;

   private final ReferenceFrame referenceFrame;
   private final YoFramePoint3D initialPosition;
   private final YoDouble intermediateZPosition;

   private final YoFramePoint3D finalPosition;
   private final YoFrameVector3D finalVelocity;

   private final YoFramePoint3D currentPosition;
   private final YoFrameVector3D currentVelocity;
   private final YoFrameVector3D currentAcceleration;

   public ParabolicWithFinalVelocityConstrainedPositionTrajectoryGenerator(String name, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this.referenceFrame = referenceFrame;
      registry = new YoRegistry(name);
      
      initialPosition = new YoFramePoint3D(name + "InitialPosition", referenceFrame, registry);
      intermediateZPosition = new YoDouble(name + "IntermediateZPosition", registry);
      
      finalPosition = new YoFramePoint3D(name + "FinalPosition", referenceFrame, registry);
      finalVelocity = new YoFrameVector3D(name + "FinalVelocity", referenceFrame, registry);

      currentPosition = new YoFramePoint3D(name + "CurrentPosition", referenceFrame, registry);
      currentVelocity = new YoFrameVector3D(name + "CurrentVelocity", referenceFrame, registry);
      currentAcceleration = new YoFrameVector3D(name + "CurrentAcceleration", referenceFrame, registry);

      currentTime = new YoDouble(name + "CurrentTime", registry);
      trajectoryTime = new YoDouble(name + "TrajectoryTime", registry);

      xPolynomial = new YoPolynomial(name + "PolynomialX", 3, registry);
      yPolynomial = new YoPolynomial(name + "PolynomialY", 3, registry);
      zPolynomial = new YoPolynomial(name + "PolynomialZ", 4, registry);

      parentRegistry.addChild(registry);
   }

   public void setTrajectoryTime(double duration)
   {
      trajectoryTime.set(duration);
   }
   
   public void setInitialConditions(FramePoint3DReadOnly initialPosition)
   {
      this.initialPosition.setMatchingFrame(initialPosition);
   }

   public void setIntermediateConditions(double intermediateZPosition)
   {
      this.intermediateZPosition.set(intermediateZPosition);  
   }
   
   public void setIntermediateConditions(YoDouble intermediateZPosition)
   {
      this.intermediateZPosition.set(intermediateZPosition.getDoubleValue());  
   }
   
   public void setFinalConditions(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity)
   {
      this.finalPosition.setMatchingFrame(finalPosition);
      this.finalVelocity.setMatchingFrame(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, FramePoint3DReadOnly initialPosition, double intermediateZPosition, FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.intermediateZPosition.set(intermediateZPosition);
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, FramePoint3DReadOnly initialPosition, YoDouble intermediateZPosition, FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.intermediateZPosition.set(intermediateZPosition.getDoubleValue());
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   public void setTrajectoryParameters(double duration, Point3DReadOnly initialPosition, double intermediatePosition, Point3DReadOnly finalPosition, Vector3DReadOnly finalVelocity)
   {
      trajectoryTime.set(duration);

      this.initialPosition.set(initialPosition);
      this.intermediateZPosition.set(intermediatePosition);
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
   }

   @Override
   public void initialize()
   {
      currentTime.set(0.0);
      double tIntermediate = trajectoryTime.getDoubleValue() / 2.0;
      xPolynomial.setQuadraticWithFinalVelocityConstraint(0.0,trajectoryTime.getDoubleValue(), initialPosition.getX(), finalPosition.getX(), finalVelocity.getX());
      yPolynomial.setQuadraticWithFinalVelocityConstraint(0.0,trajectoryTime.getDoubleValue(), initialPosition.getY(), finalPosition.getY(), finalVelocity.getY());
      zPolynomial.setCubicWithIntermediatePositionAndFinalVelocityConstraint(0.0,  tIntermediate, trajectoryTime.getDoubleValue(), initialPosition.getZ(), intermediateZPosition.getDoubleValue(), finalPosition.getZ(), finalVelocity.getZ());

      currentPosition.set(initialPosition);
      currentAcceleration.setToZero();
   }

   @Override
   public void compute(double dt)
   {
      this.currentTime.add(dt);
      double time = MathTools.clamp(currentTime.getDoubleValue(), 0.0, trajectoryTime.getDoubleValue());
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
   
   public double getTimeRemaining()
   {
      return trajectoryTime.getDoubleValue() - currentTime.getDoubleValue(); 
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return currentPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return currentVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return currentAcceleration;
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

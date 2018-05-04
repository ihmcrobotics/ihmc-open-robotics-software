package us.ihmc.robotics.math.trajectories;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class OneDoFJointQuinticTrajectoryGenerator implements OneDoFJointTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final YoDouble finalPosition;
   private final YoDouble currentPosition;
   private final YoDouble currentVelocity;
   private final YoDouble currentAcceleration;
   private final YoPolynomial polynomial;
   private final YoDouble trajectoryTime;
   private final DoubleProvider trajectoryTimeProvider;
   private final YoDouble currentTime;
   private final OneDoFJoint joint;

   public OneDoFJointQuinticTrajectoryGenerator(String namePrefix, OneDoFJoint joint, DoubleProvider trajectoryTimeProvider, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.joint = joint;
      this.polynomial = new YoPolynomial(namePrefix + "Polynomial", 6, registry);
      this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      this.currentPosition = new YoDouble(namePrefix + "CurrentPosition", registry);
      this.currentVelocity = new YoDouble(namePrefix + "CurrentVelocity", registry);
      this.currentAcceleration = new YoDouble(namePrefix + "CurrentAcceleration", registry);
      this.finalPosition = new YoDouble(namePrefix + "FinalPosition", registry);
      this.trajectoryTimeProvider = trajectoryTimeProvider;
      parentRegistry.addChild(registry);
   }

   /**
    * Desired joint angles and velocities come from reading the joints, this method can override them those position and velocity values. 
    * @param currentDesiredPosition Sets the desired joint position.
    * @param currentDesiredVelocity Sets the desired joint velocity.
    */
   @Override
   public void initialize(double initialPosition, double initialVelocity)
   {
      currentTime.set(0.0);
      this.trajectoryTime.set(trajectoryTimeProvider.getValue());
      this.polynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialPosition, initialVelocity, 0.0, finalPosition.getDoubleValue(), 0.0, 0.0);
      currentPosition.set(initialPosition);
      currentVelocity.set(initialVelocity);
      currentAcceleration.set(0.0);
   }

   @Override
   public void initialize()
   {
      initialize(joint.getQ(), joint.getQd());
   }

   @Override
   public void compute(double time)
   {
      this.currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      polynomial.compute(time);
      if (isDone() || trajectoryTime.getDoubleValue() <= 0.0)
      {
         currentPosition.set(finalPosition.getDoubleValue());
         currentVelocity.set(0.0);
         currentAcceleration.set(0.0);
      }
      else
      {
         currentPosition.set(polynomial.getPosition());
         currentVelocity.set(polynomial.getVelocity());
         currentAcceleration.set(polynomial.getAcceleration());
      }
   }

   @Override
   public boolean isDone()
   {    
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public double getValue()
   {
      return currentPosition.getDoubleValue();
   }

   @Override
   public double getVelocity()
   {
      return currentVelocity.getDoubleValue();
   }

   @Override
   public double getAcceleration()
   {
         return currentAcceleration.getDoubleValue();
   }

   public void setFinalPosition(double finalPosition)
   {
      this.finalPosition.set(finalPosition);
   }
}
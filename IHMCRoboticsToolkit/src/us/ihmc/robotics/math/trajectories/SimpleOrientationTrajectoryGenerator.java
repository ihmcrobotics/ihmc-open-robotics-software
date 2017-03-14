package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimpleOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final YoPolynomial parameterPolynomial;

   private final YoFrameQuaternion initialOrientation;
   private final YoFrameQuaternion finalOrientation;
   
   private final YoFrameQuaternion currentOrientation;
   private final YoFrameVector currentAngularVelocity;
   private final YoFrameVector currentAngularAcceleration;

   private final OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();

   public SimpleOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry);
   }

   public SimpleOrientationTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      parameterPolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);
      
      String initialOrientationName = namePrefix + "InitialOrientation";
      String finalOrientationName = namePrefix + "FinalOrientation";
      String currentOrientationName = namePrefix + "CurrentOrientation";
      String currentAngularVelocityName = namePrefix + "CurrentAngularVelocity";
      String currentAngularAccelerationName = namePrefix + "CurrentAngularAcceleration";

      if (allowMultipleFrames)
      {
         YoFrameQuaternionInMultipleFrames initialOrientation = new YoFrameQuaternionInMultipleFrames(initialOrientationName, registry, referenceFrame);
         YoFrameQuaternionInMultipleFrames finalOrientation = new YoFrameQuaternionInMultipleFrames(finalOrientationName, registry, referenceFrame);
         YoFrameQuaternionInMultipleFrames currentOrientation = new YoFrameQuaternionInMultipleFrames(currentOrientationName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentAngularVelocity = new YoFrameVectorInMultipleFrames(currentAngularVelocityName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentAngularAcceleration = new YoFrameVectorInMultipleFrames(currentAngularAccelerationName, registry, referenceFrame);

         registerMultipleFramesHolders(initialOrientation, finalOrientation, currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         this.initialOrientation = initialOrientation;
         this.finalOrientation = finalOrientation;
         this.currentOrientation = currentOrientation;
         this.currentAngularVelocity = currentAngularVelocity;
         this.currentAngularAcceleration = currentAngularAcceleration;
      }
      else
      {
         initialOrientation = new YoFrameQuaternion(initialOrientationName, referenceFrame, registry);
         finalOrientation = new YoFrameQuaternion(finalOrientationName, referenceFrame, registry);
         currentOrientation = new YoFrameQuaternion(currentOrientationName, referenceFrame, registry);
         currentAngularVelocity = new YoFrameVector(currentAngularVelocityName, referenceFrame, registry);
         currentAngularAcceleration = new YoFrameVector(currentAngularAccelerationName, referenceFrame, registry);
      }

      parentRegistry.addChild(registry);
   }

   public void setInitialOrientation(FrameOrientation initialOrientation)
   {
      this.initialOrientation.set(initialOrientation);
   }

   public void setFinalOrientation(FrameOrientation finalOrientation)
   {
      this.finalOrientation.set(finalOrientation);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
   }

   @Override
   public void initialize()
   {
      currentTime.set(0.0);
      MathTools.checkIntervalContains(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      parameterPolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
      
      currentOrientation.set(initialOrientation);
      currentAngularVelocity.setToZero();
      currentAngularAcceleration.setToZero();
   }

   @Override
   public void compute(double time)
   {
      currentTime.set(time);
      time = MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue());
      parameterPolynomial.compute(time);
      boolean shouldBeZero = isDone() || currentTime.getDoubleValue() < 0.0;
      double alphaDot = shouldBeZero ? 0.0 : parameterPolynomial.getVelocity();
      double alphaDDot = shouldBeZero ? 0.0 : parameterPolynomial.getAcceleration();

      if (!isDone())
      {
         currentOrientation.interpolate(initialOrientation, finalOrientation, parameterPolynomial.getPosition());
         orientationInterpolationCalculator.computeAngularVelocity(currentAngularVelocity, initialOrientation, finalOrientation, alphaDot);
         orientationInterpolationCalculator.computeAngularAcceleration(currentAngularAcceleration, initialOrientation, finalOrientation, alphaDDot);
      }
      else
      {
         currentOrientation.set(finalOrientation);
         currentAngularVelocity.setToZero();
         currentAngularAcceleration.setToZero();
      }
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public void getOrientation(FrameOrientation orientationToPack)
   {
      currentOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   public void getAngularVelocity(FrameVector velocityToPack)
   {
      currentAngularVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAngularAcceleration(FrameVector accelerationToPack)
   {
      currentAngularAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public String toString()
   {
      String ret = "";
      
      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent orientation: " + currentOrientation.toString();
      ret += "\nCurrent angular velocity: " + currentAngularVelocity.toString();
      ret += "\nCurrent angular acceleration: " + currentAngularAcceleration.toString();
      return ret;
   }
}

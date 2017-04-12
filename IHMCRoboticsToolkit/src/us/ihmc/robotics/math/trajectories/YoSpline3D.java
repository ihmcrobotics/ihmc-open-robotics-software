package us.ihmc.robotics.math.trajectories;

import java.util.EnumMap;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class YoSpline3D
{
   public final EnumMap<Direction, YoPolynomial> polynomials;
   private final int arcLengthCalculatorDivisions;
   private final int numberOfCoefficientsPerPolynomial;
   private final ReferenceFrame referenceFrame;
   private final DoubleYoVariable[] arcLengths;
   private final YoVariableRegistry registry;
   private final DoubleYoVariable t0;
   private final DoubleYoVariable tf;
   private final YoFramePoint position;
   private final YoFrameVector velocity;
   private final YoFrameVector acceleration;

   public YoSpline3D(int numberOfCoefficientsPerPolynomial, int arcLengthCalculatorDivisions, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry,
                     String namePrefix)
   {
      if (arcLengthCalculatorDivisions < 2)
      {
         throw new RuntimeException("arcLengthCalculatorDivisions must be at least 2");
      }

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);
      polynomials = new EnumMap<Direction, YoPolynomial>(Direction.class);
      this.numberOfCoefficientsPerPolynomial = numberOfCoefficientsPerPolynomial;
      this.arcLengthCalculatorDivisions = arcLengthCalculatorDivisions;
      this.referenceFrame = referenceFrame;
      arcLengths = new DoubleYoVariable[arcLengthCalculatorDivisions + 1];
      t0 = new DoubleYoVariable(namePrefix + "T0", registry);
      tf = new DoubleYoVariable(namePrefix + "Tf", registry);
      position = new YoFramePoint(namePrefix + "Position", referenceFrame, registry);
      velocity = new YoFrameVector(namePrefix + "Velocity", referenceFrame, registry);
      acceleration = new YoFrameVector(namePrefix + "Acceleration", referenceFrame, registry);

      for (Direction direction : Direction.values)
      {
            polynomials.put(direction, new YoPolynomial(namePrefix + "Polynomial" + direction, numberOfCoefficientsPerPolynomial, registry));
      }

      for (int i = 0; i < arcLengthCalculatorDivisions + 1; i++)
      {
         arcLengths[i] = new DoubleYoVariable(namePrefix + "ArcLength" + i, registry);
      }
   }

   public void setLinear(double t0, double tf, FramePoint p0, FramePoint pf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setLinear(t0, tf, p0.get(direction), pf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setLinearUsingInitialPositionAndVelocity(double t0, double tf, FramePoint p0, FrameVector pd0)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setLinear(t0, p0.get(direction), pd0.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setLinearUsingFinalPositionAndVelocity(double t0, double tf, FramePoint pf, FrameVector pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setLinear(tf, pf.get(direction), pdf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double t1, double tf, FramePoint p0, FrameVector pd0, FrameVector pd1, FramePoint pf,
           FrameVector pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 5);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuarticUsingIntermediateVelocity(t0, t1, tf, p0.get(direction), pd0.get(direction), pd1.get(direction),
                         pf.get(direction), pdf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double t1, double tf, FramePoint p0, FrameVector pd0, FrameVector pd1,
           FrameVector pdd1, FramePoint pf, FrameVector pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 6);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuinticUsingIntermediateVelocityAndAcceleration(t0, t1, tf, p0.get(direction), pd0.get(direction), pd1.get(direction),
                         pdd1.get(direction), pf.get(direction), pdf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setSexticUsingWaypoint(double t0, double t1, double tf, FramePoint p0, FrameVector pd0, FrameVector pdd0, FramePoint p1, FramePoint pf,
                                      FrameVector pdf, FrameVector pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 7);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setSexticUsingWaypoint(t0, t1, tf, p0.get(direction), pd0.get(direction), pdd0.get(direction), p1.get(direction),
                         pf.get(direction), pdf.get(direction), pddf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setSexticUsingWaypointVelocityAndAccelerationAndInitialAcceleration(double t0, double t1, double tf, FramePoint p0, FrameVector pd0,
           FrameVector pdd0, FrameVector pd1, FrameVector pdd1, FramePoint pf, FrameVector pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 7);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setSexticUsingWaypointVelocityAndAcceleration(t0, t1, tf, p0.get(direction), pd0.get(direction), pdd0.get(direction),
                         pd1.get(direction), pdd1.get(direction), pf.get(direction), pdf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setSexticUsingWaypointVelocityAndAccelerationAndFinalAcceleration(double t0, double t1, double tf, FramePoint p0, FrameVector pd0,
           FrameVector pd1, FrameVector pdd1, FramePoint pf, FrameVector pdf, FrameVector pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 7);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setSexticUsingWaypointVelocityAndAcceleration(tf, t1, t0, pf.get(direction), pdf.get(direction), pddf.get(direction),
                         pd1.get(direction), pdd1.get(direction), p0.get(direction), pd0.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuarticUsingInitialAcceleration(double t0, double tf, FramePoint p0, FrameVector pd0, FrameVector pdd0, FramePoint pf, FrameVector pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 5);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuartic(t0, tf, p0.get(direction), pd0.get(direction), pdd0.get(direction), pf.get(direction), pdf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tf, FramePoint p0, FrameVector pd0, FramePoint pf, FrameVector pdf, FrameVector pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 5);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuartic(tf, t0, pf.get(direction), pdf.get(direction), pddf.get(direction), p0.get(direction), pd0.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingInitialVelocity(double t0, double tf, FramePoint p0, FrameVector pd0, FramePoint pf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuadratic(t0, tf, p0.get(direction), pd0.get(direction), pf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingFinalVelocity(double t0, double tf, FramePoint p0, FramePoint pf, FrameVector pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuadratic(tf, t0, pf.get(direction), pdf.get(direction), p0.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingInitialVelocityAndAcceleration(double t0, double tf, FramePoint p0, FrameVector pd0, FrameVector pdd0)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuadraticUsingInitialAcceleration(t0, tf, p0.get(direction), pd0.get(direction), pdd0.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setCubic(double t0, double tf, FramePoint p0, FrameVector pd0, FramePoint pf, FrameVector pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 4);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setCubic(t0, tf, p0.get(direction), pd0.get(direction), pf.get(direction), pdf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tf, FramePoint p0, FrameVector pd0, FrameVector pdf, FrameVector pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 4);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setCubicUsingFinalAccelerationButNotFinalPosition(t0, tf, p0.get(direction), pd0.get(direction), pdf.get(direction),
                         pddf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   public void setQuintic(double t0, double tf, FramePoint p0, FrameVector pd0, FrameVector pdd0, FramePoint pf, FrameVector pdf, FrameVector pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 6);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuintic(t0, tf, p0.get(direction), pd0.get(direction), pdd0.get(direction), pf.get(direction), pdf.get(direction),
                         pddf.get(direction));
      }

      setYoVariables(t0, tf);
   }

   private void setYoVariables(double t0, double tf)
   {
      this.t0.set(t0);
      this.tf.set(tf);
      setArcLengths();
   }

   public void compute(double t)
   {
      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).compute(t);
      }

      for (Direction direction : Direction.values)
      {
         position.set(direction, polynomials.get(direction).getPosition());
      }

      for (Direction direction : Direction.values)
      {
         velocity.set(direction, polynomials.get(direction).getVelocity());
      }

      for (Direction direction : Direction.values)
      {
         acceleration.set(direction, polynomials.get(direction).getAcceleration());
      }
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FramePoint getPosition()
   {
      return position.getFrameTuple();
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FrameVector getVelocity()
   {
      return velocity.getFrameTuple();
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FrameVector getAcceleration()
   {
      return acceleration.getFrameTuple();
   }

   /**
    * @deprecated Creates garbage.
    */
   public FramePoint getPositionCopy()
   {
      return position.getFramePointCopy();
   }

   /**
    * @deprecated Creates garbage.
    */
   public FrameVector getVelocityCopy()
   {
      return velocity.getFrameVectorCopy();
   }
   
   /**
    * @deprecated Creates garbage.
    */
   public FrameVector getAccelerationCopy()
   {
      return acceleration.getFrameVectorCopy();
   }

   public void getPosition(FramePoint positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getVelocity(FrameVector velocityToPack)
   {
      velocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      acceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(position);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(velocity);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
   {
      accelerationToPack.set(acceleration);
   }

   public double getArcLength(double t1, double t2)
   {
      return arcLengths[getArcLengthIndex(t2)].getDoubleValue() - arcLengths[getArcLengthIndex(t1)].getDoubleValue();
   }

   public double getArcLength()
   {
      return arcLengths[arcLengths.length - 1].getDoubleValue();
   }

   private final FramePoint pi = new FramePoint();
   private final FramePoint piPlusOne = new FramePoint();

   private void setArcLengths()
   {
      pi.setToZero(referenceFrame);
      piPlusOne.setToZero(referenceFrame);
      
      arcLengths[0].set(0.0);
      double tiPlusOne = t0.getDoubleValue();
      compute(t0.getDoubleValue());
      getPosition(piPlusOne);
      double differentialDistance;
      for (int i = 0; i < arcLengthCalculatorDivisions; i++)
      {
         pi.set(piPlusOne);
         tiPlusOne = getTime(i + 1);
         compute(tiPlusOne);
         getPosition(piPlusOne);
         differentialDistance = pi.distance(piPlusOne);
         arcLengths[i + 1].set(arcLengths[i].getDoubleValue() + differentialDistance);
      }
   }

   private int getArcLengthIndex(double time)
   {
      double timeProportion = (time - t0.getDoubleValue()) / (tf.getDoubleValue() - t0.getDoubleValue());
      int arcLengthIndex = (int) Math.round(arcLengthCalculatorDivisions * timeProportion);

      return arcLengthIndex;
   }

   private double getTime(int arcLengthIndex)
   {
      double timeProportion = (double) arcLengthIndex / (double) arcLengthCalculatorDivisions;
      double time = t0.getDoubleValue() + (tf.getDoubleValue() - t0.getDoubleValue()) * timeProportion;

      return time;
   }

   public double getApproximateTimeForArcLength(double arcLength)
   {
      if (arcLength > getArcLength())
      {
         return tf.getDoubleValue() - t0.getDoubleValue();
      }
      else if (arcLength < 0)
      {
         throw new RuntimeException("arcLength must be at least 0.");
      }

      int lowerBoundIndex = 0;
      int upperBoundIndex = arcLengths.length - 1;
      int searchIndex;
      while (!(lowerBoundIndex + 1 == upperBoundIndex))
      {
         searchIndex = (upperBoundIndex + lowerBoundIndex) / 2;

         if (arcLength <= arcLengths[searchIndex].getDoubleValue())
         {
            upperBoundIndex = searchIndex;
         }
         else if (arcLength > arcLengths[searchIndex].getDoubleValue())
         {
            lowerBoundIndex = searchIndex;
         }
      }

      double lowerBound = arcLengths[lowerBoundIndex].getDoubleValue();
      double upperBound = arcLengths[upperBoundIndex].getDoubleValue();
      double proportionBetweenBounds = (arcLength - lowerBound) / (upperBound - lowerBound);
      double lowerBoundTime = getTime(lowerBoundIndex);
      double upperBoundTime = getTime(upperBoundIndex);

      return lowerBoundTime + (upperBoundTime - lowerBoundTime) * proportionBetweenBounds - t0.getDoubleValue();
   }

   public double getT0()
   {
      return t0.getDoubleValue();
   }

   public double getTf()
   {
      return tf.getDoubleValue();
   }

   public double getTotalTime()
   {
      return tf.getDoubleValue() - t0.getDoubleValue();
   }
}

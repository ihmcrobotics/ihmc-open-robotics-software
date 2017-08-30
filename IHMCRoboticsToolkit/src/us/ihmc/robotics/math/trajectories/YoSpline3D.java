package us.ihmc.robotics.math.trajectories;

import java.util.EnumMap;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;


public class YoSpline3D
{
   public final EnumMap<Direction, YoPolynomial> polynomials;
   private final int arcLengthCalculatorDivisions;
   private final int numberOfCoefficientsPerPolynomial;
   private final ReferenceFrame referenceFrame;
   private final YoDouble[] arcLengths;
   private final YoVariableRegistry registry;
   private final YoDouble t0;
   private final YoDouble tf;
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
      arcLengths = new YoDouble[arcLengthCalculatorDivisions + 1];
      t0 = new YoDouble(namePrefix + "T0", registry);
      tf = new YoDouble(namePrefix + "Tf", registry);
      position = new YoFramePoint(namePrefix + "Position", referenceFrame, registry);
      velocity = new YoFrameVector(namePrefix + "Velocity", referenceFrame, registry);
      acceleration = new YoFrameVector(namePrefix + "Acceleration", referenceFrame, registry);

      for (Direction direction : Direction.values)
      {
            polynomials.put(direction, new YoPolynomial(namePrefix + "Polynomial" + direction, numberOfCoefficientsPerPolynomial, registry));
      }

      for (int i = 0; i < arcLengthCalculatorDivisions + 1; i++)
      {
         arcLengths[i] = new YoDouble(namePrefix + "ArcLength" + i, registry);
      }
   }

   public void setLinear(double t0, double tf, FramePoint3D p0, FramePoint3D pf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setLinear(t0, tf, p0.getElement(direction.ordinal()), pf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setLinearUsingInitialPositionAndVelocity(double t0, double tf, FramePoint3D p0, FrameVector3D pd0)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);
      p0.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setLinear(t0, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setLinearUsingFinalPositionAndVelocity(double t0, double tf, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);
      pf.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setLinear(tf, pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double t1, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pd1, FramePoint3D pf,
           FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 5);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pd1.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuarticUsingIntermediateVelocity(t0, t1, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pd1.getElement(direction.ordinal()),
                         pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double t1, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pd1,
           FrameVector3D pdd1, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 6);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pd1.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);
      pdd1.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuinticUsingIntermediateVelocityAndAcceleration(t0, t1, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pd1.getElement(direction.ordinal()),
                         pdd1.getElement(direction.ordinal()), pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setSexticUsingWaypoint(double t0, double t1, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pdd0, FramePoint3D p1, FramePoint3D pf,
                                      FrameVector3D pdf, FrameVector3D pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 7);
      p0.checkReferenceFrameMatch(referenceFrame);
      p1.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdd0.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);
      pddf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setSexticUsingWaypoint(t0, t1, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pdd0.getElement(direction.ordinal()), p1.getElement(direction.ordinal()),
                         pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()), pddf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setSexticUsingWaypointVelocityAndAccelerationAndInitialAcceleration(double t0, double t1, double tf, FramePoint3D p0, FrameVector3D pd0,
           FrameVector3D pdd0, FrameVector3D pd1, FrameVector3D pdd1, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 7);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdd0.checkReferenceFrameMatch(referenceFrame);
      pd1.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);
      pdd1.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setSexticUsingWaypointVelocityAndAcceleration(t0, t1, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pdd0.getElement(direction.ordinal()),
                         pd1.getElement(direction.ordinal()), pdd1.getElement(direction.ordinal()), pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setSexticUsingWaypointVelocityAndAccelerationAndFinalAcceleration(double t0, double t1, double tf, FramePoint3D p0, FrameVector3D pd0,
           FrameVector3D pd1, FrameVector3D pdd1, FramePoint3D pf, FrameVector3D pdf, FrameVector3D pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 7);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pd1.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);
      pdd1.checkReferenceFrameMatch(referenceFrame);
      pddf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setSexticUsingWaypointVelocityAndAcceleration(tf, t1, t0, pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()), pddf.getElement(direction.ordinal()),
                         pd1.getElement(direction.ordinal()), pdd1.getElement(direction.ordinal()), p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuarticUsingInitialAcceleration(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pdd0, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 5);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdd0.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuartic(t0, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pdd0.getElement(direction.ordinal()), pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FramePoint3D pf, FrameVector3D pdf, FrameVector3D pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 5);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);
      pddf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuartic(tf, t0, pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()), pddf.getElement(direction.ordinal()), p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingInitialVelocity(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FramePoint3D pf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuadratic(t0, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingFinalVelocity(double t0, double tf, FramePoint3D p0, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuadratic(tf, t0, pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()), p0.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingInitialVelocityAndAcceleration(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pdd0)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);
      p0.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdd0.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuadraticUsingInitialAcceleration(t0, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pdd0.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setCubic(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 4);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setCubic(t0, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pdf, FrameVector3D pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 4);
      p0.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);
      pddf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setCubicUsingFinalAccelerationButNotFinalPosition(t0, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()),
                         pddf.getElement(direction.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuintic(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pdd0, FramePoint3D pf, FrameVector3D pdf, FrameVector3D pddf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 6);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdd0.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);
      pddf.checkReferenceFrameMatch(referenceFrame);

      for (Direction direction : Direction.values)
      {
         polynomials.get(direction).setQuintic(t0, tf, p0.getElement(direction.ordinal()), pd0.getElement(direction.ordinal()), pdd0.getElement(direction.ordinal()), pf.getElement(direction.ordinal()), pdf.getElement(direction.ordinal()),
                         pddf.getElement(direction.ordinal()));
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
         position.setElement(direction.ordinal(), polynomials.get(direction).getPosition());
      }

      for (Direction direction : Direction.values)
      {
         velocity.setElement(direction.ordinal(), polynomials.get(direction).getVelocity());
      }

      for (Direction direction : Direction.values)
      {
         acceleration.setElement(direction.ordinal(), polynomials.get(direction).getAcceleration());
      }
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FramePoint3D getPosition()
   {
      return position.getFrameTuple();
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FrameVector3D getVelocity()
   {
      return velocity.getFrameTuple();
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FrameVector3D getAcceleration()
   {
      return acceleration.getFrameTuple();
   }

   /**
    * @deprecated Creates garbage.
    */
   public FramePoint3D getPositionCopy()
   {
      return position.getFramePointCopy();
   }

   /**
    * @deprecated Creates garbage.
    */
   public FrameVector3D getVelocityCopy()
   {
      return velocity.getFrameVectorCopy();
   }
   
   /**
    * @deprecated Creates garbage.
    */
   public FrameVector3D getAccelerationCopy()
   {
      return acceleration.getFrameVectorCopy();
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      position.getFrameTupleIncludingFrame(positionToPack);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
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

   private final FramePoint3D pi = new FramePoint3D();
   private final FramePoint3D piPlusOne = new FramePoint3D();

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

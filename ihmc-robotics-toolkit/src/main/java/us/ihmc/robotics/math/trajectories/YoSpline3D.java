package us.ihmc.robotics.math.trajectories;

import java.util.EnumMap;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;


public class YoSpline3D
{
   public final EnumMap<Axis, YoPolynomial> polynomials;
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
      polynomials = new EnumMap<Axis, YoPolynomial>(Axis.class);
      this.numberOfCoefficientsPerPolynomial = numberOfCoefficientsPerPolynomial;
      this.arcLengthCalculatorDivisions = arcLengthCalculatorDivisions;
      this.referenceFrame = referenceFrame;
      arcLengths = new YoDouble[arcLengthCalculatorDivisions + 1];
      t0 = new YoDouble(namePrefix + "T0", registry);
      tf = new YoDouble(namePrefix + "Tf", registry);
      position = new YoFramePoint(namePrefix + "Position", referenceFrame, registry);
      velocity = new YoFrameVector(namePrefix + "Velocity", referenceFrame, registry);
      acceleration = new YoFrameVector(namePrefix + "Acceleration", referenceFrame, registry);

      for (Axis axis : Axis.values)
      {
            polynomials.put(axis, new YoPolynomial(namePrefix + "Polynomial" + axis, numberOfCoefficientsPerPolynomial, registry));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setLinear(t0, tf, p0.getElement(axis.ordinal()), pf.getElement(axis.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setLinearUsingInitialPositionAndVelocity(double t0, double tf, FramePoint3D p0, FrameVector3D pd0)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);
      p0.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setLinear(t0, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setLinearUsingFinalPositionAndVelocity(double t0, double tf, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 2);
      pf.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setLinear(tf, pf.getElement(axis.ordinal()), pdf.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuarticUsingIntermediateVelocity(t0, t1, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pd1.getElement(
               axis.ordinal()),
                                                                   pf.getElement(axis.ordinal()), pdf.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuinticUsingIntermediateVelocityAndAcceleration(t0, t1, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pd1.getElement(
               axis.ordinal()),
                                                                                  pdd1.getElement(axis.ordinal()), pf.getElement(axis.ordinal()), pdf.getElement(
                     axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setSexticUsingWaypoint(t0, t1, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pdd0.getElement(axis.ordinal()), p1.getElement(
               axis.ordinal()),
                                                      pf.getElement(axis.ordinal()), pdf.getElement(axis.ordinal()), pddf.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setSexticUsingWaypointVelocityAndAcceleration(t0, t1, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pdd0.getElement(
               axis.ordinal()),
                                                                             pd1.getElement(axis.ordinal()), pdd1.getElement(axis.ordinal()), pf.getElement(
                     axis.ordinal()), pdf.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setSexticUsingWaypointVelocityAndAcceleration(tf, t1, t0, pf.getElement(axis.ordinal()), pdf.getElement(axis.ordinal()), pddf.getElement(
               axis.ordinal()),
                                                                             pd1.getElement(axis.ordinal()), pdd1.getElement(axis.ordinal()), p0.getElement(
                     axis.ordinal()), pd0.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuartic(t0, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pdd0.getElement(axis.ordinal()), pf.getElement(
               axis.ordinal()), pdf.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuartic(tf, t0, pf.getElement(axis.ordinal()), pdf.getElement(axis.ordinal()), pddf.getElement(axis.ordinal()), p0.getElement(
               axis.ordinal()), pd0.getElement(axis.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingInitialVelocity(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FramePoint3D pf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuadratic(t0, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pf.getElement(axis.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingFinalVelocity(double t0, double tf, FramePoint3D p0, FramePoint3D pf, FrameVector3D pdf)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);
      p0.checkReferenceFrameMatch(referenceFrame);
      pf.checkReferenceFrameMatch(referenceFrame);
      pdf.checkReferenceFrameMatch(referenceFrame);

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuadratic(tf, t0, pf.getElement(axis.ordinal()), pdf.getElement(axis.ordinal()), p0.getElement(axis.ordinal()));
      }

      setYoVariables(t0, tf);
   }

   public void setQuadraticUsingInitialVelocityAndAcceleration(double t0, double tf, FramePoint3D p0, FrameVector3D pd0, FrameVector3D pdd0)
   {
      MathTools.checkEquals(numberOfCoefficientsPerPolynomial, 3);
      p0.checkReferenceFrameMatch(referenceFrame);
      pd0.checkReferenceFrameMatch(referenceFrame);
      pdd0.checkReferenceFrameMatch(referenceFrame);

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuadraticUsingInitialAcceleration(t0, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pdd0.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setCubic(t0, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pf.getElement(axis.ordinal()), pdf.getElement(
               axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setCubicUsingFinalAccelerationButNotFinalPosition(t0, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pdf.getElement(
               axis.ordinal()),
                                                                                 pddf.getElement(axis.ordinal()));
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

      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).setQuintic(t0, tf, p0.getElement(axis.ordinal()), pd0.getElement(axis.ordinal()), pdd0.getElement(axis.ordinal()), pf.getElement(
               axis.ordinal()), pdf.getElement(axis.ordinal()),
                                          pddf.getElement(axis.ordinal()));
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
      for (Axis axis : Axis.values)
      {
         polynomials.get(axis).compute(t);
      }

      for (Axis axis : Axis.values)
      {
         position.setElement(axis.ordinal(), polynomials.get(axis).getPosition());
      }

      for (Axis axis : Axis.values)
      {
         velocity.setElement(axis.ordinal(), polynomials.get(axis).getVelocity());
      }

      for (Axis axis : Axis.values)
      {
         acceleration.setElement(axis.ordinal(), polynomials.get(axis).getAcceleration());
      }
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FrameVector3DReadOnly getVelocity()
   {
      return velocity;
   }

   /**
    * GC-free but unsafe accessor.
    */
   public FrameVector3DReadOnly getAcceleration()
   {
      return acceleration;
   }

   /**
    * @deprecated Creates garbage.
    */
   public FramePoint3D getPositionCopy()
   {
      return new FramePoint3D(position);
   }

   /**
    * @deprecated Creates garbage.
    */
   public FrameVector3D getVelocityCopy()
   {
      return new FrameVector3D(velocity);
   }
   
   /**
    * @deprecated Creates garbage.
    */
   public FrameVector3D getAccelerationCopy()
   {
      return new FrameVector3D(acceleration);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(velocity);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(acceleration);
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

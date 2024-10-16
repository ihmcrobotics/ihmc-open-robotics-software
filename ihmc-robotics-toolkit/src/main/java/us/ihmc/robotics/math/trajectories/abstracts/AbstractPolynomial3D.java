package us.ihmc.robotics.math.trajectories.abstracts;

import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.core.Trajectory3DFactories;
import us.ihmc.robotics.math.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.commons.trajectories.interfaces.PolynomialBasics;
import us.ihmc.robotics.math.trajectories.interfaces.PositionTrajectoryGenerator;
import us.ihmc.commons.trajectories.yoVariables.YoPolynomial;
import us.ihmc.commons.time.TimeIntervalBasics;

import java.util.List;

/**
 * {@code YoPolynomial3D} is the simplest 3D wrapper around the 1D {@link YoPolynomial}.
 * <p>
 * Unlike {@link YoSpline3D}, {@code YoPolynomial3D} does not add extra information and is only
 * meant to simplify the interaction with polynomials when dealing with 3D trajectories.
 * </p>
 * <p>
 * The output is given in the form of {@link Point3DReadOnly}, {@link Vector3DReadOnly}, or
 * {@link Tuple3DReadOnly}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class AbstractPolynomial3D implements Polynomial3DBasics, PositionTrajectoryGenerator
{
   private final PolynomialBasics xPolynomial;
   private final PolynomialBasics yPolynomial;
   private final PolynomialBasics zPolynomial;


   private final PolynomialBasics[] polynomials;

   private double xIntegralResult = Double.NaN;
   private double yIntegralResult = Double.NaN;
   private double zIntegralResult = Double.NaN;

   private final TimeIntervalBasics timeInterval;
   private final Point3DReadOnly position;
   private final Vector3DReadOnly velocity;
   private final Vector3DReadOnly acceleration;
   private final Tuple3DReadOnly integralResult;

   private final Tuple3DBasics[] coefficients;

   public AbstractPolynomial3D(AbstractPolynomial3D other)
   {
      this(other.xPolynomial, other.yPolynomial, other.zPolynomial);
   }

   public AbstractPolynomial3D(PolynomialBasics[] yoPolynomials)
   {
      this(yoPolynomials[0], yoPolynomials[1], yoPolynomials[2]);

      if (yoPolynomials.length != 3)
         throw new RuntimeException(
               "Expected 3 YoPolynomials for representing the three axes X, Y, and Z, but had: " + yoPolynomials.length + " YoPolynomials.");
   }

   public AbstractPolynomial3D(List<? extends PolynomialBasics> yoPolynomials)
   {
      this(yoPolynomials.get(0), yoPolynomials.get(1), yoPolynomials.get(2));

      if (yoPolynomials.size() != 3)
         throw new RuntimeException(
               "Expected 3 YoPolynomials for representing the three axes X, Y, and Z, but had: " + yoPolynomials.size() + " YoPolynomials.");
   }

   public AbstractPolynomial3D(PolynomialBasics xPolynomial, PolynomialBasics yPolynomial, PolynomialBasics zPolynomial)
   {
      this.xPolynomial = xPolynomial;
      this.yPolynomial = yPolynomial;
      this.zPolynomial = zPolynomial;
      polynomials = new PolynomialBasics[] {xPolynomial, yPolynomial, zPolynomial};

      timeInterval = Trajectory3DFactories.newLinkedTimeInterval(xPolynomial, yPolynomial, zPolynomial);
      position = EuclidCoreFactories.newLinkedPoint3DReadOnly(xPolynomial::getValue, yPolynomial::getValue, zPolynomial::getValue);
      velocity = EuclidCoreFactories.newLinkedVector3DReadOnly(xPolynomial::getVelocity, yPolynomial::getVelocity, zPolynomial::getVelocity);
      acceleration = EuclidCoreFactories.newLinkedVector3DReadOnly(xPolynomial::getAcceleration, yPolynomial::getAcceleration, zPolynomial::getAcceleration);
      integralResult = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> xIntegralResult, () -> yIntegralResult, () -> zIntegralResult);

      coefficients = new Tuple3DBasics[getMaximumNumberOfCoefficients()];
      coefficients[0] = Trajectory3DFactories.newLinkedPoint3DBasics(() -> xPolynomial.getCoefficient(0),
                                                                     (d) -> xPolynomial.setCoefficient(0, d),
                                                                     () -> yPolynomial.getCoefficient(0),
                                                                     (d) -> yPolynomial.setCoefficient(0, d),
                                                                     () -> zPolynomial.getCoefficient(0),
                                                                     (d) -> zPolynomial.setCoefficient(0, d));
      for (int i = 1; i < getMaximumNumberOfCoefficients(); i++)
      {
         final int index = i;
         coefficients[index] = Trajectory3DFactories.newLinkedVector3DBasics(() -> xPolynomial.getCoefficient(index),
                                                                        (d) -> xPolynomial.setCoefficient(index, d),
                                                                        () -> yPolynomial.getCoefficient(index),
                                                                        (d) -> yPolynomial.setCoefficient(index, d),
                                                                        () -> zPolynomial.getCoefficient(index),
                                                                        (d) -> zPolynomial.setCoefficient(index, d));
      }
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }

   @Override
   public Point3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public Vector3DReadOnly getVelocity()
   {
      return velocity;
   }

   @Override
   public Vector3DReadOnly getAcceleration()
   {
      return acceleration;
   }

   @Override
   public Tuple3DBasics[] getCoefficients()
   {
      return coefficients;
   }

   public Tuple3DReadOnly getIntegral(double from, double to)
   {
      xIntegralResult = xPolynomial.getIntegral(from, to);
      yIntegralResult = yPolynomial.getIntegral(from, to);
      zIntegralResult = zPolynomial.getIntegral(from, to);
      return integralResult;
   }

   @Override
   public PolynomialBasics getAxis(int index)
   {
      return polynomials[index];
   }

   @Override
   public String toString()
   {
      return "X: " + xPolynomial.toString() + "\n" + "Y: " + yPolynomial.toString() + "\n" + "Z: " + zPolynomial.toString();
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }
}

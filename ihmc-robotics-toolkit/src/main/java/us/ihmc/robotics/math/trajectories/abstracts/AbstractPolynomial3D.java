package us.ihmc.robotics.math.trajectories.abstracts;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.core.Trajectory3DFactories;
import us.ihmc.robotics.math.trajectories.interfaces.*;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.math.trajectories.yoVariables.YoSpline3D;

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

   private final Point3DReadOnly position;
   private final Vector3DReadOnly velocity;
   private final Vector3DReadOnly acceleration;
   private final Tuple3DReadOnly integralResult;

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

      position = Trajectory3DFactories.newLinkedPoint3DReadOnly(xPolynomial, yPolynomial, zPolynomial);
      velocity = Trajectory3DFactories.newLinkedVector3DReadOnly(xPolynomial::getVelocity, yPolynomial::getVelocity, zPolynomial::getVelocity);
      acceleration = Trajectory3DFactories.newLinkedVector3DReadOnly(xPolynomial::getAcceleration, yPolynomial::getAcceleration, zPolynomial::getAcceleration);
      integralResult = Trajectory3DFactories.newLinkedPoint3DReadOnly(() -> xIntegralResult, () -> yIntegralResult, () -> zIntegralResult);
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

   public Tuple3DReadOnly getIntegral(double from, double to)
   {
      xIntegralResult = xPolynomial.getIntegral(from, to);
      yIntegralResult = yPolynomial.getIntegral(from, to);
      zIntegralResult = zPolynomial.getIntegral(from, to);
      return integralResult;
   }

   public PolynomialBasics getAxis(Axis3D axis)
   {
      return getAxis(axis.ordinal());
   }

   @Override
   public PolynomialBasics getAxis(int index)
   {
      return polynomials[index];
   }

   public void reset()
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).reset();
   }

   public void set(AbstractPolynomial3D other)
   {
      for (int i = 0; i < 3; i++)
         getAxis(i).set(other.getAxis(i));
   }

   public int getNumberOfCoefficients()
   {
      if (getNumberOfCoefficients(0) == getNumberOfCoefficients(1) && getNumberOfCoefficients(0) == getNumberOfCoefficients(2))
         return getNumberOfCoefficients(0);
      else
         return -1;
   }

   public int getNumberOfCoefficients(Axis3D dir)
   {
      return getAxis(dir.ordinal()).getNumberOfCoefficients();
   }

   public int getNumberOfCoefficients(int index)
   {
      return getAxis(index).getNumberOfCoefficients();
   }

   public void getCoefficients(int i, DMatrixRMaj coefficientsToPack)
   {
      for (int ordinal = 0; ordinal < 3; ordinal++)
      {
         coefficientsToPack.set(ordinal, 0, getAxis(ordinal).getCoefficient(i));
      }
   }

   public void offsetTrajectoryPosition(double offsetX, double offsetY, double offsetZ)
   {
      getAxis(Axis3D.X).offsetTrajectoryPosition(offsetX);
      getAxis(Axis3D.Y).offsetTrajectoryPosition(offsetY);
      getAxis(Axis3D.Z).offsetTrajectoryPosition(offsetZ);
   }

   @Override
   public String toString()
   {
      return "X: " + xPolynomial.toString() + "\n" + "Y: " + yPolynomial.toString() + "\n" + "Z: " + zPolynomial.toString();
   }
}

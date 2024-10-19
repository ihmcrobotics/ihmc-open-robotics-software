package us.ihmc.commons.trajectories.yoVariables;

import us.ihmc.commons.time.TimeIntervalBasics;
import us.ihmc.commons.trajectories.interfaces.PolynomialBasics;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D.Polynomial3DVariableHolder;
import us.ihmc.commons.trajectories.core.Trajectory3DFactories;
import us.ihmc.commons.trajectories.interfaces.Polynomial3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
import java.util.List;

/**
 * {@code YoPolynomial3D} is the simplest 3D wrapper around the 1D {@link YoPolynomial}.
 * <p>
 * {@code YoPolynomial3D} does not add extra information and is only
 * meant to simplify the interaction with polynomials when dealing with 3D trajectories.
 * </p>
 * <p>
 * The output is given in the form of {@link Point3DReadOnly}, {@link Vector3DReadOnly}, or
 * {@link Tuple3DReadOnly}.
 * </p>
 *
 * @author Robert Griffin
 */
public class YoPolynomial3D implements Polynomial3DBasics, Polynomial3DVariableHolder
{
   private final PolynomialBasics[] polynomials;

   private final TimeIntervalBasics timeInterval;
   private final Point3DReadOnly position;
   private final Vector3DReadOnly velocity;
   private final Vector3DReadOnly acceleration;

   private final Tuple3DBasics[] coefficients;

   public YoPolynomial3D(String name, int maximumNumberOfCoefficients, YoRegistry registry)
   {
      this(new YoPolynomial(name + "X", maximumNumberOfCoefficients, registry),
           new YoPolynomial(name + "Y", maximumNumberOfCoefficients, registry),
           new YoPolynomial(name + "Z", maximumNumberOfCoefficients, registry));
   }

   public YoPolynomial3D(YoPolynomial xPolynomial, YoPolynomial yPolynomial, YoPolynomial zPolynomial)
   {
      polynomials = new PolynomialBasics[] {xPolynomial, yPolynomial, zPolynomial};

      timeInterval = Trajectory3DFactories.newLinkedTimeInterval(xPolynomial, yPolynomial, zPolynomial);
      position = EuclidCoreFactories.newLinkedPoint3DReadOnly(xPolynomial::getValue, yPolynomial::getValue, zPolynomial::getValue);
      velocity = EuclidCoreFactories.newLinkedVector3DReadOnly(xPolynomial::getVelocity, yPolynomial::getVelocity, zPolynomial::getVelocity);
      acceleration = EuclidCoreFactories.newLinkedVector3DReadOnly(xPolynomial::getAcceleration, yPolynomial::getAcceleration, zPolynomial::getAcceleration);

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

   public static List<YoPolynomial3D> createYoPolynomial3DList(List<YoPolynomial> xPolynomial, List<YoPolynomial> yPolynomial, List<YoPolynomial> zPolynomial)
   {
      return Arrays.asList(createYoPolynomial3DArray(xPolynomial, yPolynomial, zPolynomial));
   }

   public static YoPolynomial3D[] createYoPolynomial3DArray(List<YoPolynomial> xPolynomial, List<YoPolynomial> yPolynomial, List<YoPolynomial> zPolynomial)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      YoPolynomial3D[] yoPolynomial3Ds = new YoPolynomial3D[xPolynomial.size()];

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds[i] = new YoPolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i));
      }
      return yoPolynomial3Ds;
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }

   public YoPolynomial getYoPolynomial(Axis3D axis)
   {
      return getYoPolynomial(axis.ordinal());
   }

   public YoPolynomial getYoPolynomial(int index)
   {
      return (YoPolynomial) getAxis(index);
   }

   @Override
   public YoPolynomial getYoPolynomialX()
   {
      return getYoPolynomial(Axis3D.X.ordinal());
   }

   @Override
   public YoPolynomial getYoPolynomialY()
   {
      return getYoPolynomial(Axis3D.Y.ordinal());
   }

   @Override
   public YoPolynomial getYoPolynomialZ()
   {
      return getYoPolynomial(Axis3D.Z.ordinal());
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

   @Override
   public PolynomialBasics getAxis(int index)
   {
      return polynomials[index];
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   @Override
   public String toString()
   {
      return "X: " + getAxis(Axis3D.X).toString() + "\n" + "Y: " + getAxis(Axis3D.Y).toString() + "\n" + "Z: " + getAxis(Axis3D.Z).toString();
   }
}

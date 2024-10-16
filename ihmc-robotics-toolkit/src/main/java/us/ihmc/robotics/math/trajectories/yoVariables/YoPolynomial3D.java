package us.ihmc.robotics.math.trajectories.yoVariables;

import us.ihmc.commons.trajectories.yoVariables.YoPolynomial;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D.Polynomial3DVariableHolder;
import us.ihmc.robotics.math.trajectories.abstracts.AbstractPolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
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
public class YoPolynomial3D extends AbstractPolynomial3D implements Polynomial3DVariableHolder
{
   protected final YoPolynomial xPolynomial;
   protected final YoPolynomial yPolynomial;
   protected final YoPolynomial zPolynomial;

   public YoPolynomial3D(String name, int maximumNumberOfCoefficients, YoRegistry registry)
   {
      this(new YoPolynomial(name + "X", maximumNumberOfCoefficients, registry),
           new YoPolynomial(name + "Y", maximumNumberOfCoefficients, registry),
           new YoPolynomial(name + "Z", maximumNumberOfCoefficients, registry));
   }

   public YoPolynomial3D(YoPolynomial xPolynomial, YoPolynomial yPolynomial, YoPolynomial zPolynomial)
   {
      super(xPolynomial, yPolynomial, zPolynomial);

      this.xPolynomial = xPolynomial;
      this.yPolynomial = yPolynomial;
      this.zPolynomial = zPolynomial;
   }

   public static YoPolynomial3D[] createYoPolynomial3DArray(YoPolynomial[] xPolynomial, YoPolynomial[] yPolynomial, YoPolynomial[] zPolynomial)
   {
      return createYoPolynomial3DArray(Arrays.asList(xPolynomial), Arrays.asList(yPolynomial), Arrays.asList(zPolynomial));
   }

   public static List<YoPolynomial3D> createYoPolynomial3DList(YoPolynomial[] xPolynomial, YoPolynomial[] yPolynomial, YoPolynomial[] zPolynomial)
   {
      return Arrays.asList(createYoPolynomial3DArray(xPolynomial, yPolynomial, zPolynomial));
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
}

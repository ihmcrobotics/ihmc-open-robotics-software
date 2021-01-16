package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolynomial3D.Polynomial3DVariableHolder;
import us.ihmc.yoVariables.registry.YoRegistry;

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
public class YoPolynomial3D extends Axis3DPositionTrajectoryGenerator implements Polynomial3DInterface, Polynomial3DVariableHolder
{
   protected final YoPolynomial xPolynomial;
   protected final YoPolynomial yPolynomial;
   protected final YoPolynomial zPolynomial;

   private double xIntegralResult = Double.NaN;
   private double yIntegralResult = Double.NaN;
   private double zIntegralResult = Double.NaN;

   private final Tuple3DReadOnly integralResult = new Tuple3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xIntegralResult;
      }

      @Override
      public double getY()
      {
         return yIntegralResult;
      }

      @Override
      public double getZ()
      {
         return zIntegralResult;
      }
   };

   public YoPolynomial3D(String name, int maximumNumberOfCoefficients, YoRegistry registry)
   {
      this(new YoPolynomial(name + "X", maximumNumberOfCoefficients, registry),
           new YoPolynomial(name + "Y", maximumNumberOfCoefficients, registry),
           new YoPolynomial(name + "Z", maximumNumberOfCoefficients, registry));
   }

   public YoPolynomial3D(YoPolynomial[] yoPolynomials)
   {
      this(yoPolynomials[0], yoPolynomials[1], yoPolynomials[2]);

      if (yoPolynomials.length != 3)
         throw new RuntimeException(
               "Expected 3 YoPolynomials for representing the three axes X, Y, and Z, but had: " + yoPolynomials.length + " YoPolynomials.");
   }

   public YoPolynomial3D(List<YoPolynomial> yoPolynomials)
   {
      this(yoPolynomials.get(0), yoPolynomials.get(1), yoPolynomials.get(2));

      if (yoPolynomials.size() != 3)
         throw new RuntimeException(
               "Expected 3 YoPolynomials for representing the three axes X, Y, and Z, but had: " + yoPolynomials.size() + " YoPolynomials.");
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
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      YoPolynomial3D[] yoPolynomial3Ds = new YoPolynomial3D[xPolynomial.length];

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds[i] = new YoPolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i]);
      }
      return yoPolynomial3Ds;
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

   public static List<YoPolynomial3D> createYoPolynomial3DList(YoPolynomial[] xPolynomial, YoPolynomial[] yPolynomial, YoPolynomial[] zPolynomial)
   {
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<YoPolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.length);

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds.add(new YoPolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i]));
      }
      return yoPolynomial3Ds;
   }

   public static List<YoPolynomial3D> createYoPolynomial3DList(List<YoPolynomial> xPolynomial, List<YoPolynomial> yPolynomial, List<YoPolynomial> zPolynomial)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<YoPolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.size());

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds.add(new YoPolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i)));
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


   public Tuple3DReadOnly getIntegral(double from, double to)
   {
      xIntegralResult = xPolynomial.getIntegral(from, to);
      yIntegralResult = yPolynomial.getIntegral(from, to);
      zIntegralResult = zPolynomial.getIntegral(from, to);
      return integralResult;
   }

   public YoPolynomial getYoPolynomial(Axis3D axis)
   {
      return getYoPolynomial(axis.ordinal());
   }

   @Override
   public PolynomialInterface getAxis(int index)
   {
      return getYoPolynomial(index);
   }

   public YoPolynomial getYoPolynomial(int index)
   {
      switch (index)
      {
         case 0:
            return getYoPolynomialX();
         case 1:
            return getYoPolynomialY();
         case 2:
            return getYoPolynomialZ();
         default:
            throw new IndexOutOfBoundsException(Integer.toString(index));
      }
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

   public void reset()
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).reset();
   }

   public void set(YoPolynomial3D other)
   {
      for (int i = 0; i < 3; i++)
         getYoPolynomial(i).set(other.getYoPolynomial(i));
   }

   @Override
   public String toString()
   {
      return "X: " + xPolynomial.toString() + "\n" + "Y: " + yPolynomial.toString() + "\n" + "Z: " + zPolynomial.toString();
   }
}

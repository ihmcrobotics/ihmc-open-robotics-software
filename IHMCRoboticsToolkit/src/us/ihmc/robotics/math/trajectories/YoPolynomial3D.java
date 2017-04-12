package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.Direction;

/**
 * {@code YoPolynomial3D} is the simplest 3D wrapper around the 1D {@link YoPolynomial}.
 * <p>
 * Unlike {@link YoSpline3D}, {@code YoPolynomial3D} does not add extra information and is only
 * meant to simplify the interaction with polynomials when dealing with 3D trajectories.
 * </p>
 * <p>
 * The output is given in the form of {@link Point3DReadOnly}, {@link Vector3DReadOnly}, or {@link Tuple3DReadOnly}.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class YoPolynomial3D
{
   private final YoPolynomial xPolynomial;
   private final YoPolynomial yPolynomial;
   private final YoPolynomial zPolynomial;

   private final Point3DReadOnly position = new Point3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xPolynomial.getPosition();
      }

      @Override
      public double getY()
      {
         return yPolynomial.getPosition();
      }

      @Override
      public double getZ()
      {
         return zPolynomial.getPosition();
      }
   };

   private final Vector3DReadOnly velocity = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xPolynomial.getVelocity();
      }

      @Override
      public double getY()
      {
         return yPolynomial.getVelocity();
      }

      @Override
      public double getZ()
      {
         return zPolynomial.getVelocity();
      }
   };

   private final Vector3DReadOnly acceleration = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xPolynomial.getAcceleration();
      }

      @Override
      public double getY()
      {
         return yPolynomial.getAcceleration();
      }

      @Override
      public double getZ()
      {
         return zPolynomial.getAcceleration();
      }
   };

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

   public YoPolynomial3D(String name, int maximumNumberOfCoefficients, YoVariableRegistry registry)
   {
      xPolynomial = new YoPolynomial(name + "X", maximumNumberOfCoefficients, registry);
      yPolynomial = new YoPolynomial(name + "Y", maximumNumberOfCoefficients, registry);
      zPolynomial = new YoPolynomial(name + "Z", maximumNumberOfCoefficients, registry);
   }

   public YoPolynomial3D(YoPolynomial xPolynomial, YoPolynomial yPolynomial, YoPolynomial zPolynomial)
   {
      this.xPolynomial = xPolynomial;
      this.yPolynomial = yPolynomial;
      this.zPolynomial = zPolynomial;
   }

   public YoPolynomial3D(YoPolynomial[] yoPolynomials)
   {
      if (yoPolynomials.length != 3)
         throw new RuntimeException("Expected 3 YoPolynomials for representing the three axes X, Y, and Z, but had: " + yoPolynomials.length
               + " YoPolynomials.");

      this.xPolynomial = yoPolynomials[0];
      this.yPolynomial = yoPolynomials[1];
      this.zPolynomial = yoPolynomials[2];
   }

   public YoPolynomial3D(List<YoPolynomial> yoPolynomials)
   {
      if (yoPolynomials.size() != 3)
         throw new RuntimeException("Expected 3 YoPolynomials for representing the three axes X, Y, and Z, but had: " + yoPolynomials.size()
               + " YoPolynomials.");

      this.xPolynomial = yoPolynomials.get(0);
      this.yPolynomial = yoPolynomials.get(1);
      this.zPolynomial = yoPolynomials.get(2);
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

   public void compute(double t)
   {
      xPolynomial.compute(t);
      yPolynomial.compute(t);
      zPolynomial.compute(t);
   }

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public Vector3DReadOnly getVelocity()
   {
      return velocity;
   }

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

   public YoPolynomial getYoPolynomial(Direction direction)
   {
      return getYoPolynomial(direction.ordinal());
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

   public YoPolynomial getYoPolynomialX()
   {
      return xPolynomial;
   }

   public YoPolynomial getYoPolynomialY()
   {
      return yPolynomial;
   }

   public YoPolynomial getYoPolynomialZ()
   {
      return zPolynomial;
   }

   public void setConstant(Point3DReadOnly z)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setConstant(z.getElement(index));
   }

   public void setCubic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setCubic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal,
                                                           Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0.getElement(index), zFinal.getElement(index),
                                                                            zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                                           Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                                            zFinal.getElement(index));
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdFinal,
                                                                 Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                  zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, Point3DReadOnly z0,
                                               Point3DReadOnly zIntermediate1, Point3DReadOnly zIntermediate2, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0.getElement(index),
                                                                zIntermediate1.getElement(index), zIntermediate2.getElement(index), zFinal.getElement(index));
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                          Point3DReadOnly zIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                           zIntermediate.getElement(index), zFinal.getElement(index),
                                                                                           zdFinal.getElement(index));
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0,
                                                                            Vector3DReadOnly zd0, Point3DReadOnly zIntermediate, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0.getElement(index),
                                                                                             zd0.getElement(index), zIntermediate.getElement(index),
                                                                                             zFinal.getElement(index));
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                       Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                        zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setLinear(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setLinear(t0, tFinal, z0.getElement(index), zf.getElement(index));
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                        Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                        Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                         zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                         zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   public void setQuadratic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuadratic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index));
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuadraticUsingInitialAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index));
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Point3DReadOnly zIntermediate,
                                                  Point3DReadOnly zFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0.getElement(index), zIntermediate.getElement(index),
                                                                   zFinal.getElement(index));
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, Point3DReadOnly z0, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setQuartic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zFinal,
                          Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuartic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zFinal.getElement(index),
                                           zdFinal.getElement(index));

   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zFinal,
                                                Vector3DReadOnly zdFinal, Vector3DReadOnly zddFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuarticUsingFinalAcceleration(t0, tFinal, z0.getElement(index), zd0.getElement(index), zFinal.getElement(index),
                                                                 zdFinal.getElement(index), zddFinal.getElement(index));
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                   Vector3DReadOnly zdIntermediate, Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                    zdIntermediate.getElement(index), zFinal.getElement(index), zdFinal.getElement(index));

   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zMid, Point3DReadOnly zFinal,
                                       Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuarticUsingMidPoint(t0, tFinal, z0.getElement(index), zd0.getElement(index), zMid.getElement(index),
                                                        zFinal.getElement(index), zdFinal.getElement(index));

   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zFinal,
                                                      Vector3DReadOnly zdIntermediate1)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index),
                                                                       zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                       zFinal.getElement(index), zdIntermediate1.getElement(index));

   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Point3DReadOnly zIntermediate,
                                       Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zIntermediate.getElement(index),
                                                        zf.getElement(index), zdf.getElement(index));
   }

   public void setQuintic(double t0, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zf, Vector3DReadOnly zdf,
                          Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuintic(t0, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index), zf.getElement(index),
                                           zdf.getElement(index), zddf.getElement(index));
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                      Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                       zIntermediate0.getElement(index), zIntermediate1.getElement(index), zf.getElement(index),
                                                       zdf.getElement(index));
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                                  Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate, Point3DReadOnly zFinal,
                                                                  Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                                   zdIntermediate.getElement(index), zddIntermediate.getElement(index),
                                                                                   zFinal.getElement(index), zdFinal.getElement(index));
   }

   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                       Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                        zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index));
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                        Point3DReadOnly zIntermediate, Vector3DReadOnly zdIntermediate, Point3DReadOnly zf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                         zIntermediate.getElement(index), zdIntermediate.getElement(index), zf.getElement(index));
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                         Point3DReadOnly zIntermediate0, Vector3DReadOnly zdIntermediate0, Point3DReadOnly zIntermediate1, Vector3DReadOnly zdIntermediate1,
                         Point3DReadOnly zf, Vector3DReadOnly zdf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                          zIntermediate0.getElement(index), zdIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                          zdIntermediate1.getElement(index), zf.getElement(index), zdf.getElement(index));

   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, Point3DReadOnly z0,
                                                    Vector3DReadOnly zd0, Vector3DReadOnly zdd0, Point3DReadOnly zIntermediate0, Point3DReadOnly zIntermediate1,
                                                    Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                     zdd0.getElement(index), zIntermediate0.getElement(index), zIntermediate1.getElement(index),
                                                                     zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0, Vector3DReadOnly zdd0,
                                      Point3DReadOnly zIntermediate, Point3DReadOnly zf, Vector3DReadOnly zdf, Vector3DReadOnly zddf)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index), zdd0.getElement(index),
                                                       zIntermediate.getElement(index), zf.getElement(index), zdf.getElement(index), zddf.getElement(index));

   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, Point3DReadOnly z0, Vector3DReadOnly zd0,
                                                             Vector3DReadOnly zdd0, Vector3DReadOnly zdIntermediate, Vector3DReadOnly zddIntermediate,
                                                             Point3DReadOnly zFinal, Vector3DReadOnly zdFinal)
   {
      for (int index = 0; index < 3; index++)
         getYoPolynomial(index).setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getElement(index), zd0.getElement(index),
                                                                              zdd0.getElement(index), zdIntermediate.getElement(index),
                                                                              zddIntermediate.getElement(index), zFinal.getElement(index),
                                                                              zdFinal.getElement(index));

   }
}

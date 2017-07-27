package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2dReadOnly;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

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
public class YoFramePolynomial3D extends YoPolynomial3D
{
   private ReferenceFrame referenceFrame;

   private final FramePoint framePosition = new FramePoint();
   private final FrameVector frameVelocity = new FrameVector();
   private final FrameVector frameAcceleration = new FrameVector();

   public YoFramePolynomial3D(String name, int maximumNumberOfCoefficients, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(name, maximumNumberOfCoefficients, registry);
      this.referenceFrame = referenceFrame;
   }

   public YoFramePolynomial3D(YoPolynomial xPolynomial, YoPolynomial yPolynomial, YoPolynomial zPolynomial, ReferenceFrame referenceFrame)
   {
      super(xPolynomial, yPolynomial, zPolynomial);
      this.referenceFrame = referenceFrame;
   }

   public YoFramePolynomial3D(YoPolynomial[] yoPolynomials, ReferenceFrame referenceFrame)
   {
      super(yoPolynomials);
      this.referenceFrame = referenceFrame;
   }

   public YoFramePolynomial3D(List<YoPolynomial> yoPolynomials, ReferenceFrame referenceFrame)
   {
      super(yoPolynomials);
      this.referenceFrame = referenceFrame;
   }

   public static YoFramePolynomial3D[] createYoFramePolynomial3DArray(YoPolynomial[] xPolynomial, YoPolynomial[] yPolynomial, YoPolynomial[] zPolynomial,
                                                                      ReferenceFrame referenceFrame)
   {
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      YoFramePolynomial3D[] yoPolynomial3Ds = new YoFramePolynomial3D[xPolynomial.length];

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds[i] = new YoFramePolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i], referenceFrame);
      }
      return yoPolynomial3Ds;
   }

   public static YoFramePolynomial3D[] createYoFramePolynomial3DArray(List<YoPolynomial> xPolynomial, List<YoPolynomial> yPolynomial,
                                                                      List<YoPolynomial> zPolynomial, ReferenceFrame referenceFrame)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      YoFramePolynomial3D[] yoPolynomial3Ds = new YoFramePolynomial3D[xPolynomial.size()];

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds[i] = new YoFramePolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i), referenceFrame);
      }

      return yoPolynomial3Ds;
   }

   public static List<YoFramePolynomial3D> createYoFramePolynomial3DList(YoPolynomial[] xPolynomial, YoPolynomial[] yPolynomial, YoPolynomial[] zPolynomial,
                                                                         ReferenceFrame referenceFrame)
   {
      if (xPolynomial.length != yPolynomial.length || xPolynomial.length != zPolynomial.length)
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<YoFramePolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.length);

      for (int i = 0; i < xPolynomial.length; i++)
      {
         yoPolynomial3Ds.add(new YoFramePolynomial3D(xPolynomial[i], yPolynomial[i], zPolynomial[i], referenceFrame));
      }
      return yoPolynomial3Ds;
   }

   public static List<YoFramePolynomial3D> createYoFramePolynomial3DList(List<YoPolynomial> xPolynomial, List<YoPolynomial> yPolynomial,
                                                                         List<YoPolynomial> zPolynomial, ReferenceFrame referenceFrame)
   {
      if (xPolynomial.size() != yPolynomial.size() || xPolynomial.size() != zPolynomial.size())
         throw new RuntimeException("Cannot handle different number of polynomial for the different axes.");

      List<YoFramePolynomial3D> yoPolynomial3Ds = new ArrayList<>(xPolynomial.size());

      for (int i = 0; i < xPolynomial.size(); i++)
      {
         yoPolynomial3Ds.add(new YoFramePolynomial3D(xPolynomial.get(i), yPolynomial.get(i), zPolynomial.get(i), referenceFrame));
      }
      return yoPolynomial3Ds;
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void set(YoFramePolynomial3D other)
   {
      setReferenceFrame(other.getReferenceFrame());

      xPolynomial.set(other.getYoPolynomialX());
      yPolynomial.set(other.getYoPolynomialY());
      zPolynomial.set(other.getYoPolynomialZ());
   }

   public void setConstant(FramePoint z)
   {
      z.checkReferenceFrameMatch(referenceFrame);

      setConstant(z.getPoint());
   }

   public void setCubic(double t0, double tFinal, FramePoint z0, FramePoint zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubic(t0, tFinal, z0.getPoint(), zFinal.getPoint());
   }

   public void setCubic(double t0, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setCubic(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, FramePoint z0, FramePoint zFinal, FrameVector zdFinal, FrameVector zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0.getPoint(), zFinal.getPoint(), zdFinal.getVector(), zddFinal.getVector());
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0, FramePoint zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zFinal.getPoint());
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdFinal, FrameVector zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0.getPoint(), zd0.getVector(), zdFinal.getVector(), zddFinal.getVector());
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, FramePoint z0, FramePoint zIntermediate1,
                                               FramePoint zIntermediate2, FramePoint zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zIntermediate2.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0.getPoint(), zIntermediate1.getPoint(), zIntermediate2.getPoint(), zFinal.getPoint());
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, FramePoint z0, FramePoint zIntermediate1, FramePoint zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0.getPoint(), zIntermediate1.getPoint(), zFinal.getPoint());
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint z0,
                                                                          FramePoint zIntermediate, FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0.getPoint(), zIntermediate.getPoint(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0,
                                                                            FramePoint zIntermediate, FramePoint zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zIntermediate.getPoint(), zFinal.getPoint());
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setLinear(double t0, double tFinal, FramePoint z0, FramePoint zf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);

      setLinear(t0, tFinal, z0.getPoint(), zf.getPoint());
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zIntermediate0,
                        FrameVector zdIntermediate0, FramePoint zIntermediate1, FrameVector zdIntermediate1, FramePoint zf, FrameVector zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getPoint(), zd0.getVector(), zIntermediate0.getPoint(), zdIntermediate0.getVector(),
               zIntermediate1.getPoint(), zdIntermediate1.getVector(), zf.getPoint(), zdf.getVector());
   }

   public void setQuadratic(double t0, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setQuadratic(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint());
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);

      setQuadraticUsingInitialAcceleration(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector());
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, FramePoint z0, FramePoint zIntermediate, FramePoint zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0.getPoint(), zIntermediate.getPoint(), zFinal.getPoint());
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, FramePoint z0, FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0.getPoint(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuartic(double t0, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0, FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuartic(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zFinal, FrameVector zdFinal, FrameVector zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingFinalAcceleration(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint(), zdFinal.getVector(), zddFinal.getVector());
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdIntermediate,
                                                   FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdIntermediate.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zMid, FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zMid.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingMidPoint(t0, tFinal, z0.getPoint(), zd0.getVector(), zMid.getPoint(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint z0,
                                                      FramePoint zIntermediate0, FramePoint zIntermediate1, FramePoint zFinal, FrameVector zdIntermediate1)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate1.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0.getPoint(), zIntermediate0.getPoint(), zIntermediate1.getPoint(),
                                             zFinal.getPoint(), zdIntermediate1.getVector());
   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zIntermediate, FramePoint zf,
                                       FrameVector zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zIntermediate.getPoint(), zf.getPoint(), zdf.getVector());
   }

   public void setQuintic(double t0, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0, FramePoint zf, FrameVector zdf, FrameVector zddf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);
      zddf.checkReferenceFrameMatch(referenceFrame);

      setQuintic(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zf.getPoint(), zdf.getVector(), zddf.getVector());
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint z0, FrameVector zd0,
                                      FramePoint zIntermediate0, FramePoint zIntermediate1, FramePoint zf, FrameVector zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0.getPoint(), zd0.getVector(), zIntermediate0.getPoint(), zIntermediate1.getPoint(),
                             zf.getPoint(), zdf.getVector());
   }

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0,
                                                                  FrameVector zdIntermediate, FrameVector zddIntermediate, FramePoint zFinal,
                                                                  FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate.checkReferenceFrameMatch(referenceFrame);
      zddIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdIntermediate.getVector(),
                                                         zddIntermediate.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0,
                                       FramePoint zIntermediate, FramePoint zf, FrameVector zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zIntermediate.getPoint(), zf.getPoint(), zdf.getVector());
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0,
                                        FramePoint zIntermediate, FrameVector zdIntermediate, FramePoint zf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);

      setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zIntermediate.getPoint(),
                               zdIntermediate.getVector(), zf.getPoint());
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint z0, FrameVector zd0, FramePoint zIntermediate0,
                         FrameVector zdIntermediate0, FramePoint zIntermediate1, FrameVector zdIntermediate1, FramePoint zf, FrameVector zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0.getPoint(), zd0.getVector(), zIntermediate0.getPoint(), zdIntermediate0.getVector(),
                zIntermediate1.getPoint(), zdIntermediate1.getVector(), zf.getPoint(), zdf.getVector());
   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint z0, FrameVector zd0,
                                                    FrameVector zdd0, FramePoint zIntermediate0, FramePoint zIntermediate1, FramePoint zf, FrameVector zdf,
                                                    FrameVector zddf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);
      zddf.checkReferenceFrameMatch(referenceFrame);

      setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(),
                                           zIntermediate0.getPoint(), zIntermediate1.getPoint(), zf.getPoint(), zdf.getVector(), zddf.getVector());
   }

   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0,
                                      FramePoint zIntermediate, FramePoint zf, FrameVector zdf, FrameVector zddf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);
      zddf.checkReferenceFrameMatch(referenceFrame);

      setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zIntermediate.getPoint(), zf.getPoint(),
                             zdf.getVector(), zddf.getVector());
   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, FramePoint z0, FrameVector zd0, FrameVector zdd0,
                                                             FrameVector zdIntermediate, FrameVector zddIntermediate, FramePoint zFinal, FrameVector zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate.checkReferenceFrameMatch(referenceFrame);
      zddIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zdIntermediate.getVector(),
                                                    zddIntermediate.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }


   public FramePoint getFramePosition()
   {
      framePosition.setToZero(referenceFrame);
      framePosition.set(getPosition());
      return framePosition;
   }

   public void getFramePosition(FramePoint positionToPack)
   {
      positionToPack.setToZero(referenceFrame);
      positionToPack.set(getPosition());
   }

   public FrameVector getFrameVelocity()
   {
      frameVelocity.setToZero(referenceFrame);
      frameVelocity.set(getVelocity());
      return frameVelocity;
   }

   public void getFrameVelocity(FrameVector velocityToPack)
   {
      velocityToPack.setToZero(referenceFrame);
      velocityToPack.set(getVelocity());
   }

   public FrameVector getFrameAcceleration()
   {
      frameAcceleration.setToZero(referenceFrame);
      frameAcceleration.set(getAcceleration());
      return frameAcceleration;
   }

   public void getFrameAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.setToZero(referenceFrame);
      accelerationToPack.set(getAcceleration());
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

}

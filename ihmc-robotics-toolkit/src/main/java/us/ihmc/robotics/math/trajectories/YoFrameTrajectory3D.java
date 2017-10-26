package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoFrameTrajectory3D extends YoTrajectory3D implements ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;

   private final FramePoint3D framePosition = new FramePoint3D();
   private final FrameVector3D frameVelocity = new FrameVector3D();
   private final FrameVector3D frameAcceleration = new FrameVector3D();

   public YoFrameTrajectory3D(String name, int maximumNumberOfCoefficients, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      super(name, maximumNumberOfCoefficients, registry);
      this.referenceFrame = referenceFrame;
   }

   public YoFrameTrajectory3D(YoTrajectory xTrajectory, YoTrajectory yTrajectory, YoTrajectory zTrajectory, ReferenceFrame referenceFrame)
   {
      super(xTrajectory, yTrajectory, zTrajectory);
      this.referenceFrame = referenceFrame;
   }

   public YoFrameTrajectory3D(YoTrajectory[] yoTrajectories, ReferenceFrame referenceFrame)
   {
      super(yoTrajectories);
      this.referenceFrame = referenceFrame;
   }

   public YoFrameTrajectory3D(List<YoTrajectory> yoTrajectories, ReferenceFrame referenceFrame)
   {
      super(yoTrajectories);
      this.referenceFrame = referenceFrame;
   }

   public static YoFrameTrajectory3D[] createYoFrameTrajectory3DArray(YoTrajectory[] xTrajectory, YoTrajectory[] yTrajectory, YoTrajectory[] zTrajectory,
                                                                      ReferenceFrame referenceFrame)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      YoFrameTrajectory3D[] yoTrajectory3Ds = new YoFrameTrajectory3D[xTrajectory.length];

      for (int i = 0; i < xTrajectory.length; i++)
      {
         yoTrajectory3Ds[i] = new YoFrameTrajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i], referenceFrame);
      }
      return yoTrajectory3Ds;
   }

   public static YoFrameTrajectory3D[] createYoFrameTrajectory3DArray(List<YoTrajectory> xTrajectory, List<YoTrajectory> yTrajectory,
                                                                      List<YoTrajectory> zTrajectory, ReferenceFrame referenceFrame)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      YoFrameTrajectory3D[] yoTrajectory3Ds = new YoFrameTrajectory3D[xTrajectory.size()];

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         yoTrajectory3Ds[i] = new YoFrameTrajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i), referenceFrame);
      }

      return yoTrajectory3Ds;
   }

   public static List<YoFrameTrajectory3D> createYoFrameTrajectory3DList(YoTrajectory[] xTrajectory, YoTrajectory[] yTrajectory, YoTrajectory[] zTrajectory,
                                                                         ReferenceFrame referenceFrame)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<YoFrameTrajectory3D> yoTrajectory3Ds = new ArrayList<>(xTrajectory.length);

      for (int i = 0; i < xTrajectory.length; i++)
      {
         yoTrajectory3Ds.add(new YoFrameTrajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i], referenceFrame));
      }
      return yoTrajectory3Ds;
   }

   public static List<YoFrameTrajectory3D> createYoFrameTrajectoryl3DList(List<YoTrajectory> xTrajectory, List<YoTrajectory> yTrajectory,
                                                                          List<YoTrajectory> zTrajectory, ReferenceFrame referenceFrame)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<YoFrameTrajectory3D> yoTrajectory3Ds = new ArrayList<>(xTrajectory.size());

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         yoTrajectory3Ds.add(new YoFrameTrajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i), referenceFrame));
      }
      return yoTrajectory3Ds;
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void set(YoFrameTrajectory3D other)
   {
      setReferenceFrame(other.getReferenceFrame());

      xTrajectory.set(other.getYoTrajectoryX());
      yTrajectory.set(other.getYoTrajectoryY());
      zTrajectory.set(other.getYoTrajectoryZ());
   }

   public void setConstant(double t0, double tFinal, FramePoint3D z0)
   {
      checkReferenceFrameMatch(z0);
      setConstant(t0, tFinal, z0.getPoint());
   }

   public void setCubic(double t0, double tFinal, FramePoint3D z0, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubic(t0, tFinal, z0.getPoint(), zFinal.getPoint());
   }

   public void setCubic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setCubic(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setCubicBezier(double t0, double tFinal, FramePoint3D z0, FramePoint3D zR1, FramePoint3D zR2, FramePoint3D zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zR1);
      checkReferenceFrameMatch(zR2);
      checkReferenceFrameMatch(zFinal);

      setCubicBezier(t0, tFinal, z0.getPoint(), zR1.getPoint(), zR2.getPoint(), zFinal.getPoint());
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, FramePoint3D z0, FramePoint3D zFinal, FrameVector3D zdFinal,
                                                           FrameVector3D zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0.getPoint(), zFinal.getPoint(), zdFinal.getVector(), zddFinal.getVector());
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zFinal.getPoint());
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdFinal,
                                                                 FrameVector3D zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0.getPoint(), zd0.getVector(), zdFinal.getVector(), zddFinal.getVector());
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, FramePoint3D z0, FramePoint3D zIntermediate1,
                                               FramePoint3D zIntermediate2, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zIntermediate2.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0.getPoint(), zIntermediate1.getPoint(), zIntermediate2.getPoint(),
                                      zFinal.getPoint());
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, FramePoint3D z0, FramePoint3D zIntermediate1, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0.getPoint(), zIntermediate1.getPoint(), zFinal.getPoint());
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint3D z0,
                                                                          FramePoint3D zIntermediate, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0.getPoint(), zIntermediate.getPoint(), zFinal.getPoint(),
                                                                 zdFinal.getVector());
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                                                            FramePoint3D zIntermediate, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zIntermediate.getPoint(),
                                                                   zFinal.getPoint());
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal,
                                                                       FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setLinear(double t0, double tFinal, FramePoint3D z0, FramePoint3D zf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);

      setLinear(t0, tFinal, z0.getPoint(), zf.getPoint());
   }

   public void setNonic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zIntermediate0,
                        FrameVector3D zdIntermediate0, FramePoint3D zIntermediate1, FrameVector3D zdIntermediate1, FramePoint3D zf, FrameVector3D zdf)
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

   public void setQuadratic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setQuadratic(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint());
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);

      setQuadraticUsingInitialAcceleration(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector());
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FramePoint3D zIntermediate, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0.getPoint(), zIntermediate.getPoint(), zFinal.getPoint());
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, FramePoint3D z0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0.getPoint(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuartic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuartic(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal, FrameVector3D zdFinal,
                                                FrameVector3D zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingFinalAcceleration(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint(), zdFinal.getVector(), zddFinal.getVector());
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdIntermediate,
                                                   FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdIntermediate.getVector(), zFinal.getPoint(),
                                          zdFinal.getVector());
   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zMid, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zMid.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingMidPoint(t0, tFinal, z0.getPoint(), zd0.getVector(), zMid.getPoint(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0,
                                                      FramePoint3D zIntermediate0, FramePoint3D zIntermediate1, FramePoint3D zFinal, FrameVector3D zdIntermediate1)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate1.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0.getPoint(), zIntermediate0.getPoint(), zIntermediate1.getPoint(),
                                             zFinal.getPoint(), zdIntermediate1.getVector());
   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zIntermediate, FramePoint3D zf,
                                       FrameVector3D zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zIntermediate.getPoint(), zf.getPoint(), zdf.getVector());
   }

   public void setQuintic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0, FramePoint3D zf, FrameVector3D zdf, FrameVector3D zddf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);
      zddf.checkReferenceFrameMatch(referenceFrame);

      setQuintic(t0, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zf.getPoint(), zdf.getVector(), zddf.getVector());
   }

   public void setQuinticTwoWaypoints(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                      FramePoint3D zIntermediate0, FramePoint3D zIntermediate1, FramePoint3D zf, FrameVector3D zdf)
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

   public void setQuinticUsingIntermediateVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                                                  FrameVector3D zdIntermediate, FrameVector3D zddIntermediate, FramePoint3D zFinal,
                                                                  FrameVector3D zdFinal)
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

   public void setQuinticUsingWayPoint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0,
                                       FramePoint3D zIntermediate, FramePoint3D zf, FrameVector3D zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0.getPoint(), zd0.getVector(), zdd0.getVector(), zIntermediate.getPoint(), zf.getPoint(),
                              zdf.getVector());
   }

   public void setQuinticUsingWayPoint2(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0,
                                        FramePoint3D zIntermediate, FrameVector3D zdIntermediate, FramePoint3D zf)
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

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zIntermediate0,
                         FrameVector3D zdIntermediate0, FramePoint3D zIntermediate1, FrameVector3D zdIntermediate1, FramePoint3D zf, FrameVector3D zdf)
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

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                                    FrameVector3D zdd0, FramePoint3D zIntermediate0, FramePoint3D zIntermediate1, FramePoint3D zf, FrameVector3D zdf,
                                                    FrameVector3D zddf)
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

   public void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      setQuinticWithZeroTerminalAcceleration(t0, tFinal, z0.getPoint(), zd0.getVector(), zFinal.getPoint(), zdFinal.getVector());
   }

   public void setSexticUsingWaypoint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0,
                                      FramePoint3D zIntermediate, FramePoint3D zf, FrameVector3D zdf, FrameVector3D zddf)
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

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0,
                                                             FrameVector3D zdIntermediate, FrameVector3D zddIntermediate, FramePoint3D zFinal, FrameVector3D zdFinal)
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
   
   public void setDirectly(Axis axis, DenseMatrix64F coefficients)
   {
      getYoTrajectory(axis).setDirectly(coefficients);
   }

   public FramePoint3D getFramePosition()
   {
      framePosition.setToZero(referenceFrame);
      framePosition.set(getPosition());
      return framePosition;
   }

   public void getFramePosition(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(referenceFrame);
      positionToPack.set(getPosition());
   }
   
   public void getFramePositionInitial(FramePoint3D positionToPack)
   {
      compute(xTrajectory.getInitialTime());
      PrintTools.debug("Initial time = " + xTrajectory.getInitialTime());
      positionToPack.setToZero(referenceFrame);
      positionToPack.set(getPosition());
   }
   
   public void getFramePositionFinal(FramePoint3D positionToPack)
   {
      compute(xTrajectory.getFinalTime());
      PrintTools.debug("Final time = " + xTrajectory.getFinalTime());
      positionToPack.setToZero(referenceFrame);
      positionToPack.set(getPosition());
   }

   public FrameVector3D getFrameVelocity()
   {
      frameVelocity.setToZero(referenceFrame);
      frameVelocity.set(getVelocity());
      return frameVelocity;
   }

   public void getFrameVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setToZero(referenceFrame);
      velocityToPack.set(getVelocity());
   }

   public FrameVector3D getFrameAcceleration()
   {
      frameAcceleration.setToZero(referenceFrame);
      frameAcceleration.set(getAcceleration());
      return frameAcceleration;
   }

   public void getFrameAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setToZero(referenceFrame);
      accelerationToPack.set(getAcceleration());
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public void checkReferenceFrameMatch(ReferenceFrameHolder referenceFrameHolder) throws ReferenceFrameMismatchException
   {
      getReferenceFrame().checkReferenceFrameMatch(referenceFrameHolder.getReferenceFrame());
   }

   @Override
   public void checkReferenceFrameMatch(ReferenceFrame frame) throws ReferenceFrameMismatchException
   {
      getReferenceFrame().checkReferenceFrameMatch(referenceFrame);
   }

   public int getNumberOfCoefficients()
   {
      return Math.max(Math.max(xTrajectory.getNumberOfCoefficients(), yTrajectory.getNumberOfCoefficients()), zTrajectory.getNumberOfCoefficients());
   }
   
   public void getDerivative(int order, double x, FrameTuple3D<?, ?> dQuantity)
   {
      dQuantity.setIncludingFrame(referenceFrame, xTrajectory.getDerivative(order, x), yTrajectory.getDerivative(order, x), zTrajectory.getDerivative(order, x));
   }

   public void getDerivative(YoFrameTrajectory3D dervTraj, int order)
   {
      checkReferenceFrameMatch(dervTraj);
      super.getDerivative(dervTraj, order);
   }
   
   public void getStartPoint(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(referenceFrame);
      super.getStartPoint(positionToPack.getPoint());
   }
   
   public void getEndPoint(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(referenceFrame);
      super.getEndPoint(positionToPack.getPoint());
   }
   
   public boolean isValidTrajectory()
   {
      return super.isValidTrajectory();
   }
   
   public void set(FrameTrajectory3D trajToCopy)
   {
      checkReferenceFrameMatch(trajToCopy);
      super.set(trajToCopy);
   }
}

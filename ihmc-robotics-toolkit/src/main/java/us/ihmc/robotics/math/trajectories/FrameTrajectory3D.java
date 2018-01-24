package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

public class FrameTrajectory3D extends Trajectory3D implements ReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;

   private final FramePoint3D framePosition = new FramePoint3D();
   private final FrameVector3D frameVelocity = new FrameVector3D();
   private final FrameVector3D frameAcceleration = new FrameVector3D();

   public FrameTrajectory3D(int maximumNumberOfCoefficients, ReferenceFrame referenceFrame)
   {
      super(maximumNumberOfCoefficients);
      this.referenceFrame = referenceFrame;
   }

   public FrameTrajectory3D(Trajectory xTrajectory, Trajectory yTrajectory, Trajectory zTrajectory, ReferenceFrame referenceFrame)
   {
      super(xTrajectory, yTrajectory, zTrajectory);
      this.referenceFrame = referenceFrame;
   }

   public FrameTrajectory3D(Trajectory[] trajectories, ReferenceFrame referenceFrame)
   {
      super(trajectories);
      this.referenceFrame = referenceFrame;
   }

   public FrameTrajectory3D(List<Trajectory> trajectories, ReferenceFrame referenceFrame)
   {
      super(trajectories);
      this.referenceFrame = referenceFrame;
   }

   public static FrameTrajectory3D[] createFrameTrajectory3DArray(Trajectory[] xTrajectory, Trajectory[] yTrajectory, Trajectory[] zTrajectory,
                                                                  ReferenceFrame referenceFrame)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      FrameTrajectory3D[] trajectory3Ds = new FrameTrajectory3D[xTrajectory.length];

      for (int i = 0; i < xTrajectory.length; i++)
      {
         trajectory3Ds[i] = new FrameTrajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i], referenceFrame);
      }
      return trajectory3Ds;
   }

   public static FrameTrajectory3D[] createFrameTrajectory3DArray(List<Trajectory> xTrajectory, List<Trajectory> yTrajectory,
                                                                  List<Trajectory> zTrajectory, ReferenceFrame referenceFrame)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      FrameTrajectory3D[] trajectory3Ds = new FrameTrajectory3D[xTrajectory.size()];

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         trajectory3Ds[i] = new FrameTrajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i), referenceFrame);
      }

      return trajectory3Ds;
   }

   public static List<FrameTrajectory3D> createFrameTrajectory3DList(Trajectory[] xTrajectory, Trajectory[] yTrajectory, Trajectory[] zTrajectory,
                                                                     ReferenceFrame referenceFrame)
   {
      if (xTrajectory.length != yTrajectory.length || xTrajectory.length != zTrajectory.length)
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<FrameTrajectory3D> trajectory3Ds = new ArrayList<>(xTrajectory.length);

      for (int i = 0; i < xTrajectory.length; i++)
      {
         trajectory3Ds.add(new FrameTrajectory3D(xTrajectory[i], yTrajectory[i], zTrajectory[i], referenceFrame));
      }
      return trajectory3Ds;
   }

   public static List<FrameTrajectory3D> createFrameTrajectoryl3DList(List<Trajectory> xTrajectory, List<Trajectory> yTrajectory,
                                                                      List<Trajectory> zTrajectory, ReferenceFrame referenceFrame)
   {
      if (xTrajectory.size() != yTrajectory.size() || xTrajectory.size() != zTrajectory.size())
         throw new RuntimeException("Cannot handle different number of trajectories for the different axes.");

      List<FrameTrajectory3D> trajectory3Ds = new ArrayList<>(xTrajectory.size());

      for (int i = 0; i < xTrajectory.size(); i++)
      {
         trajectory3Ds.add(new FrameTrajectory3D(xTrajectory.get(i), yTrajectory.get(i), zTrajectory.get(i), referenceFrame));
      }
      return trajectory3Ds;
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void set(FrameTrajectory3D other)
   {
      setReferenceFrame(other.getReferenceFrame());

      xTrajectory.set(other.getTrajectoryX());
      yTrajectory.set(other.getTrajectoryY());
      zTrajectory.set(other.getTrajectoryZ());
   }

   public void setConstant(double t0, double tFinal, FramePoint3D z0)
   {
      checkReferenceFrameMatch(z0);
      setConstant(t0, tFinal, z0);
   }

   public void setCubic(double t0, double tFinal, FramePoint3D z0, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubic(t0, tFinal, z0, zFinal);
   }

   public void setCubic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubic(t0, tFinal, z0, zd0, zFinal, zdFinal);
   }

   public void setCubicBezier(double t0, double tFinal, FramePoint3D z0, FramePoint3D zR1, FramePoint3D zR2, FramePoint3D zFinal)
   {
      checkReferenceFrameMatch(z0);
      checkReferenceFrameMatch(zR1);
      checkReferenceFrameMatch(zR2);
      checkReferenceFrameMatch(zFinal);

      super.setCubicBezier(t0, tFinal, z0, zR1, zR2, zFinal);
   }

   public void setCubicInitialPositionThreeFinalConditions(double t0, double tFinal, FramePoint3D z0, FramePoint3D zFinal, FrameVector3D zdFinal,
                                                           FrameVector3D zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubicInitialPositionThreeFinalConditions(t0, tFinal, z0, zFinal, zdFinal, zddFinal);
   }

   public void setCubicThreeInitialConditionsFinalPosition(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0,
                                                           FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubicThreeInitialConditionsFinalPosition(t0, tFinal, z0, zd0, zdd0, zFinal);
   }

   public void setCubicUsingFinalAccelerationButNotFinalPosition(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdFinal,
                                                                 FrameVector3D zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubicUsingFinalAccelerationButNotFinalPosition(t0, tFinal, z0, zd0, zdFinal, zddFinal);
   }

   public void setCubicUsingIntermediatePoints(double t0, double tIntermediate1, double tIntermediate2, double tFinal, FramePoint3D z0,
                                               FramePoint3D zIntermediate1, FramePoint3D zIntermediate2, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zIntermediate2.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubicUsingIntermediatePoints(t0, tIntermediate1, tIntermediate2, tFinal, z0, zIntermediate1, zIntermediate2,
                                      zFinal);
   }

   public void setCubicUsingIntermediatePoint(double t0, double tIntermediate1, double tFinal, FramePoint3D z0, FramePoint3D zIntermediate1,
                                              FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubicUsingIntermediatePoint(t0, tIntermediate1, tFinal, z0, zIntermediate1, zFinal);
   }

   public void setCubicWithIntermediatePositionAndFinalVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint3D z0,
                                                                          FramePoint3D zIntermediate, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubicWithIntermediatePositionAndFinalVelocityConstraint(t0, tIntermediate, tFinal, z0, zIntermediate, zFinal,
                                                                 zdFinal);
   }

   public void setCubicWithIntermediatePositionAndInitialVelocityConstraint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                                                            FramePoint3D zIntermediate, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      super.setCubicWithIntermediatePositionAndInitialVelocityConstraint(t0, tIntermediate, tFinal, z0, zd0, zIntermediate,
                                                                   zFinal);
   }

   public void setInitialPositionVelocityZeroFinalHighOrderDerivatives(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal,
                                                                       FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setInitialPositionVelocityZeroFinalHighOrderDerivatives(t0, tFinal, z0, zd0, zFinal, zdFinal);
   }

   public void setLinear(double t0, double tFinal, FramePoint3D z0, FramePoint3D zf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);

      super.setLinear(t0, tFinal, z0, zf);
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

      super.setNonic(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zdIntermediate0,
               zIntermediate1, zdIntermediate1, zf, zdf);
   }

   public void setQuadratic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuadratic(t0, tFinal, z0, zd0, zFinal);
   }

   public void setQuadraticUsingInitialAcceleration(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);

      super.setQuadraticUsingInitialAcceleration(t0, tFinal, z0, zd0, zdd0);
   }

   public void setQuadraticUsingIntermediatePoint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FramePoint3D zIntermediate,
                                                  FramePoint3D zFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuadraticUsingIntermediatePoint(t0, tIntermediate, tFinal, z0, zIntermediate, zFinal);
   }

   public void setQuadraticWithFinalVelocityConstraint(double t0, double tFinal, FramePoint3D z0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuadraticWithFinalVelocityConstraint(t0, tFinal, z0, zFinal, zdFinal);
   }

   public void setQuartic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuartic(t0, tFinal, z0, zd0, zdd0, zFinal, zdFinal);
   }

   public void setQuarticUsingFinalAcceleration(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal, FrameVector3D zdFinal,
                                                FrameVector3D zddFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);
      zddFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuarticUsingFinalAcceleration(t0, tFinal, z0, zd0, zFinal, zdFinal, zddFinal);
   }

   public void setQuarticUsingIntermediateVelocity(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                                   FrameVector3D zdIntermediate, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuarticUsingIntermediateVelocity(t0, tIntermediate, tFinal, z0, zd0, zdIntermediate, zFinal,
                                          zdFinal);
   }

   public void setQuarticUsingMidPoint(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zMid, FramePoint3D zFinal,
                                       FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zMid.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuarticUsingMidPoint(t0, tFinal, z0, zd0, zMid, zFinal, zdFinal);
   }

   public void setQuarticUsingOneIntermediateVelocity(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0,
                                                      FramePoint3D zIntermediate0, FramePoint3D zIntermediate1, FramePoint3D zFinal,
                                                      FrameVector3D zdIntermediate1)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate1.checkReferenceFrameMatch(referenceFrame);

      super.setQuarticUsingOneIntermediateVelocity(t0, tIntermediate0, tIntermediate1, tFinal, z0, zIntermediate0, zIntermediate1,
                                             zFinal, zdIntermediate1);
   }

   public void setQuarticUsingWayPoint(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zIntermediate,
                                       FramePoint3D zf, FrameVector3D zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      super.setQuarticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zIntermediate, zf, zdf);
   }

   public void setQuintic(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FrameVector3D zdd0, FramePoint3D zf, FrameVector3D zdf,
                          FrameVector3D zddf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);
      zddf.checkReferenceFrameMatch(referenceFrame);

      super.setQuintic(t0, tFinal, z0, zd0, zdd0, zf, zdf, zddf);
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

      super.setQuinticTwoWaypoints(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zIntermediate1,
                             zf, zdf);
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

      super.setQuinticUsingIntermediateVelocityAndAcceleration(t0, tIntermediate, tFinal, z0, zd0, zdIntermediate,
                                                         zddIntermediate, zFinal, zdFinal);
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

      super.setQuinticUsingWayPoint(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zf,
                              zdf);
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

      super.setQuinticUsingWayPoint2(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate,
                               zdIntermediate, zf);
   }

   public void setSeptic(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                         FramePoint3D zIntermediate0, FrameVector3D zdIntermediate0, FramePoint3D zIntermediate1, FrameVector3D zdIntermediate1,
                         FramePoint3D zf, FrameVector3D zdf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);

      super.setSeptic(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zIntermediate0, zdIntermediate0,
                zIntermediate1, zdIntermediate1, zf, zdf);
   }

   public void setSepticInitialAndFinalAcceleration(double t0, double tIntermediate0, double tIntermediate1, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                                    FrameVector3D zdd0, FramePoint3D zIntermediate0, FramePoint3D zIntermediate1, FramePoint3D zf,
                                                    FrameVector3D zdf, FrameVector3D zddf)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate0.checkReferenceFrameMatch(referenceFrame);
      zIntermediate1.checkReferenceFrameMatch(referenceFrame);
      zf.checkReferenceFrameMatch(referenceFrame);
      zdf.checkReferenceFrameMatch(referenceFrame);
      zddf.checkReferenceFrameMatch(referenceFrame);

      super.setSepticInitialAndFinalAcceleration(t0, tIntermediate0, tIntermediate1, tFinal, z0, zd0, zdd0,
                                           zIntermediate0, zIntermediate1, zf, zdf, zddf);
   }

   public void setQuinticWithZeroTerminalAcceleration(double t0, double tFinal, FramePoint3D z0, FrameVector3D zd0, FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setQuinticWithZeroTerminalAcceleration(t0, tFinal, z0, zd0, zFinal, zdFinal);
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

      super.setSexticUsingWaypoint(t0, tIntermediate, tFinal, z0, zd0, zdd0, zIntermediate, zf,
                             zdf, zddf);
   }

   public void setSexticUsingWaypointVelocityAndAcceleration(double t0, double tIntermediate, double tFinal, FramePoint3D z0, FrameVector3D zd0,
                                                             FrameVector3D zdd0, FrameVector3D zdIntermediate, FrameVector3D zddIntermediate,
                                                             FramePoint3D zFinal, FrameVector3D zdFinal)
   {
      z0.checkReferenceFrameMatch(referenceFrame);
      zd0.checkReferenceFrameMatch(referenceFrame);
      zdd0.checkReferenceFrameMatch(referenceFrame);
      zdIntermediate.checkReferenceFrameMatch(referenceFrame);
      zddIntermediate.checkReferenceFrameMatch(referenceFrame);
      zFinal.checkReferenceFrameMatch(referenceFrame);
      zdFinal.checkReferenceFrameMatch(referenceFrame);

      super.setSexticUsingWaypointVelocityAndAcceleration(t0, tIntermediate, tFinal, z0, zd0, zdd0, zdIntermediate,
                                                    zddIntermediate, zFinal, zdFinal);
   }

   public void setDirectly(Axis axis, DenseMatrix64F coefficients)
   {
      getTrajectory(axis).setDirectly(coefficients);
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
      positionToPack.setToZero(referenceFrame);
      positionToPack.set(getPosition());
   }

   public void getFramePositionFinal(FramePoint3D positionToPack)
   {
      compute(xTrajectory.getFinalTime());
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

   public void getDerivative(int order, double x, FrameTuple3DBasics dQuantity)
   {
      dQuantity.setIncludingFrame(referenceFrame, xTrajectory.getDerivative(order, x), yTrajectory.getDerivative(order, x),
                                  zTrajectory.getDerivative(order, x));
   }

   public void getDerivative(FrameTrajectory3D dervTraj, int order)
   {
      checkReferenceFrameMatch(dervTraj);
      super.getDerivative(dervTraj, order);
   }

   public void getStartPoint(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(referenceFrame);
      super.getStartPoint(positionToPack);
   }
   
   public void getEndPoint(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(referenceFrame);
      super.getEndPoint(positionToPack);
   }
   
   public boolean isValidTrajectory()
   {
      return super.isValidTrajectory();
   }
}

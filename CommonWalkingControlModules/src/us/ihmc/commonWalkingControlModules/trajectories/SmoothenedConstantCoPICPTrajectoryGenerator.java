package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.humanoidRobot.footstep.FootstepUtils;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;


public class SmoothenedConstantCoPICPTrajectoryGenerator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFramePoint2d desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector2d desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);
   private final int lookahead;

   public SmoothenedConstantCoPICPTrajectoryGenerator(YoVariableRegistry parentRegistry, int lookahead)
   {
      this.lookahead = lookahead;
      parentRegistry.addChild(registry);
   }

   public void compute(List<Footstep> footsteps, double w0, double singleSupportTime, double doubleSupportTime, double transferIntoWalkingTime)
   {
      List<FramePoint2d> equivalentConstantCoPs = getEquivalentConstantCoPs(footsteps, lookahead);
      double steppingTime = singleSupportTime + doubleSupportTime;

      getDesiredICPWaypoints(equivalentConstantCoPs, w0, steppingTime);
   }

   public FramePoint2d getDesiredICP()
   {
      return desiredICP.getFramePoint2dCopy();
   }

   public FrameVector2d getDesiredICPVelocity()
   {
      return desiredICPVelocity.getFrameVector2dCopy();
   }

   public List<FramePoint2d> getEquivalentConstantCoPs(List<Footstep> footsteps, int lookahead)
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>();

      for (int i = 0; i < Math.min(lookahead, footsteps.size()); i++)
      {
         Footstep footstep = footsteps.get(i);
         FramePoint centerOfFootstep = FootstepUtils.getCenterOfFootstep(footstep);
         ret.add(centerOfFootstep.toFramePoint2d());
      }
      return ret;
   }

   public List<FramePoint2d> getDesiredICPWaypoints(List<FramePoint2d> equivalentConstantCoPs, double w0, double steppingTime)
   {

      int numberOfEquivalentConstantCoPs = equivalentConstantCoPs.size();
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>();

      for (int i = 0; i < numberOfEquivalentConstantCoPs; i++)
      {
         FramePoint2d dummyFramePoint2d = new FramePoint2d(ReferenceFrame.getWorldFrame());
         ret.add(dummyFramePoint2d);
      }

      ret.get(numberOfEquivalentConstantCoPs - 1).set(equivalentConstantCoPs.get(numberOfEquivalentConstantCoPs - 1));

      for (int i = numberOfEquivalentConstantCoPs - 2; i >= 0; i--)
      {
         FramePoint2d cmp = equivalentConstantCoPs.get(i);

         FramePoint2d icp = ret.get(i + 1);
         ret.get(i).set(EquivalentConstantCoPCalculator.computeICPPositionWithConstantCMP(icp, cmp, -steppingTime, w0));
      }

      return ret;
   }

   public DenseMatrix64F computeDoubleSupportpolynomialParams(List<FramePoint2d> equivalentConstantCoPs, List<FramePoint2d> desiredICPWaypoints, double w0,
         double steppingTime, double doubleSupportTime)
   {
      // Check, if the CoPs and the ICPs are in the same frames
      for (int i = 0; i < equivalentConstantCoPs.size(); i++)
      {
         equivalentConstantCoPs.get(i).checkReferenceFrameMatch(desiredICPWaypoints.get(i));
      }

      double smootheningStartTime = steppingTime - doubleSupportTime / 2.0;
      FramePoint2d initialDoubleSupportICP = EquivalentConstantCoPCalculator.computeICPPositionWithConstantCMP(desiredICPWaypoints.get(0),
            equivalentConstantCoPs.get(0), smootheningStartTime, w0);

      FrameVector2d initialDoubleSupportICPVelocity = EquivalentConstantCoPCalculator.computeICPVelocityWithConstantCMP(desiredICPWaypoints.get(0),
            equivalentConstantCoPs.get(0), smootheningStartTime, w0);

      double smootheningEndTime = doubleSupportTime / 2.0;
      FramePoint2d finalDoubleSupportICP = EquivalentConstantCoPCalculator.computeICPPositionWithConstantCMP(desiredICPWaypoints.get(1),
            equivalentConstantCoPs.get(1), smootheningEndTime, w0);

      FrameVector2d finalDoubleSupportICPVelocity = EquivalentConstantCoPCalculator.computeICPVelocityWithConstantCMP(desiredICPWaypoints.get(1),
            equivalentConstantCoPs.get(1), smootheningStartTime, w0);

      DenseMatrix64F initialDoubleSupportICPposMat = new DenseMatrix64F(1, 2, true, 0, 0);
      DenseMatrix64F initialDoubleSupportICPVelocityMat = new DenseMatrix64F(1, 2, true, 0, 0);
      DenseMatrix64F finalDoubleSupportICPMat = new DenseMatrix64F(1, 2, true, 0, 0);
      DenseMatrix64F finalDoubleSupportICPVelocityMat = new DenseMatrix64F(1, 2, true, 0, 0);

      MatrixTools.transformFramePoint2dIntoRowVector(initialDoubleSupportICPposMat, initialDoubleSupportICP);
      MatrixTools.transformFrameVector2dIntoRowVector(initialDoubleSupportICPVelocityMat, initialDoubleSupportICPVelocity);
      MatrixTools.transformFramePoint2dIntoRowVector(finalDoubleSupportICPMat, finalDoubleSupportICP);
      MatrixTools.transformFrameVector2dIntoRowVector(finalDoubleSupportICPVelocityMat, finalDoubleSupportICPVelocity);

      double doubleSupportTimePow2 = Math.pow(doubleSupportTime, 2);
      double doubleSupportTimePow3 = Math.pow(doubleSupportTime, 3);

      // Calculate time-dependency matrix for polynomial calculation (part of inversion problem)
      DenseMatrix64F TimeBoundaryConditionMatrix = new DenseMatrix64F(4, 4, true, -2, 1, 2, 1, 3 * doubleSupportTime, -2 * doubleSupportTime, -3
            * doubleSupportTime, -doubleSupportTime, 0, doubleSupportTimePow2, 0, 0, -doubleSupportTimePow3, 0, 0, 0);

      //        = [                -2,         1,          2,              1; ...
      //                            3*t2,       -2*t2,      -3*t2,     -t2; ...
      //                            0,          t2_2,       0,         0; ...
      //                            -t2_3,      0,          0,       0];

      // Calculate DenominatorMatrix, so that T =  TimeBoundaryConditionMatrix*DenominatorMatrix, so that PolynomialParams = T*StateBoundaryConditionMatrix
      double denominator1 = -1 / doubleSupportTimePow3;
      double denominator2 = 1 / doubleSupportTimePow2;
      DenseMatrix64F DenominatorMatrix = CommonOps.diag(denominator1, denominator2, denominator1, denominator2);

      // Calculate PolynomialParams via inversion (see description above)
      DenseMatrix64F StateBoundaryConditionMatrix = new DenseMatrix64F(4, 2);
      MatrixTools.setMatrixRowToVector(0, StateBoundaryConditionMatrix, initialDoubleSupportICPposMat);
      MatrixTools.setMatrixRowToVector(1, StateBoundaryConditionMatrix, initialDoubleSupportICPVelocityMat);
      MatrixTools.setMatrixRowToVector(2, StateBoundaryConditionMatrix, finalDoubleSupportICPMat);
      MatrixTools.setMatrixRowToVector(3, StateBoundaryConditionMatrix, finalDoubleSupportICPVelocityMat);

      DenseMatrix64F tempResultMatrix = new DenseMatrix64F(4, 2);
      DenseMatrix64F tempParamMatrix = new DenseMatrix64F(4, 2);
      DenseMatrix64F paramMatrix = new DenseMatrix64F(2, 4);
      CommonOps.mult(DenominatorMatrix, StateBoundaryConditionMatrix, tempResultMatrix);
      CommonOps.mult(TimeBoundaryConditionMatrix, tempResultMatrix, tempParamMatrix);
      CommonOps.transpose(tempParamMatrix);
      paramMatrix.set(tempParamMatrix);

      return paramMatrix;
   }

   public void calcDCMandECMPofTime(List<FramePoint2d> equivalentConstantCoPs, List<FramePoint2d> desiredICPWaypoints, double w0, double currentTime,
         double steppingTime, double doubleSupportTime, boolean isSingeSupport, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity,
         FramePoint2d desiredEquivalentConstantCOP, DenseMatrix64F paramMatrix)
   {
      if (isSingeSupport)
      {
         double singleSupportComputationTime = currentTime + 0.5 * doubleSupportTime;

         desiredICP = EquivalentConstantCoPCalculator.computeICPPositionWithConstantCMP(desiredICPWaypoints.get(0), equivalentConstantCoPs.get(0),
               singleSupportComputationTime, w0);
         desiredICPVelocity = EquivalentConstantCoPCalculator.computeICPVelocityWithConstantCMP(desiredICPWaypoints.get(0), equivalentConstantCoPs.get(0),
               singleSupportComputationTime, w0);
         desiredEquivalentConstantCOP.set(equivalentConstantCoPs.get(0));
      }
      else
      {
         double timePow3 = Math.pow(currentTime, 3.0);
         double timePow2 = Math.pow(currentTime, 2.0);
         DenseMatrix64F tempVector = new DenseMatrix64F(2, 1);
         DenseMatrix64F desiredICPMat = new DenseMatrix64F(2, 1);
         DenseMatrix64F desiredICPVelocityMat = new DenseMatrix64F(2, 1);
         DenseMatrix64F desiredEquivalentConstantCOPMat = new DenseMatrix64F(2, 1);

         DenseMatrix64F desiredICPPositionTimeBoundaryConditionVector = new DenseMatrix64F(4, 1, true, timePow3, timePow2, currentTime, 1);
         DenseMatrix64F desiredICPVelocityTimeBoundaryConditionVector = new DenseMatrix64F(4, 1, true, 3 * timePow2, 2 * currentTime, 1, 0);

         CommonOps.mult(paramMatrix, desiredICPPositionTimeBoundaryConditionVector, desiredICPMat);
         CommonOps.mult(paramMatrix, desiredICPVelocityTimeBoundaryConditionVector, desiredICPVelocityMat);
         CommonOps.scale(-w0, desiredICPVelocityMat, tempVector);
         CommonOps.add(desiredICPMat, tempVector, desiredEquivalentConstantCOPMat);

         MatrixTools.transformColumnVectorIntoFramePoint2d(desiredICPMat, desiredICP);

         MatrixTools.transformColumnVectorIntoFrameVector2d(desiredICPVelocityMat, desiredICPVelocity);

         MatrixTools.transformColumnVectorIntoFramePoint2d(desiredEquivalentConstantCOPMat, desiredEquivalentConstantCOP);
      }
   }
}
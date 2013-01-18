package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.HashMap;
import java.util.LinkedHashMap;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.LeeGoswamiForceOptimizerNative;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiForceOptimizer
{
   private static final int VECTOR3D_LENGTH = 3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable wk = new DoubleYoVariable("wk", registry);    // TODO: better name
   private final DoubleYoVariable epsilonF = new DoubleYoVariable("epsilonF", registry);    // TODO: better name

   private final ReferenceFrame centerOfMassFrame;
   private final int nSupportVectors;

   private final DenseMatrix64F phi;
   private final DenseMatrix64F xi;
   private final DenseMatrix64F rho;

   private final DenseMatrix64F betaBlock;
   private final DenseMatrix64F rhoBlock;
   private final DenseMatrix64F forceMatrix;

   // column major:
   private final double[] phiArray;    // note: this is phi as in the paper *without* the bottom epsilonF * 1 block
   private final double[] xiArray;    // note: this is xi as in the paper *without* the zeros appended at the end

   private final LeeGoswamiForceOptimizerNative leeGoswamiForceOptimizerNative;

   private final SpatialForceVector wrenchError;

   public LeeGoswamiForceOptimizer(ReferenceFrame centerOfMassFrame, int nSupportVectors, YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      this.nSupportVectors = nSupportVectors;

      int maxNContacts = 2;
      int nRows = 2 * VECTOR3D_LENGTH;
      int nColumns = maxNContacts * nSupportVectors;

      phi = new DenseMatrix64F(nRows, nColumns);
      xi = new DenseMatrix64F(nRows, 1);
      rho = new DenseMatrix64F(nColumns, 1);
      betaBlock = new DenseMatrix64F(VECTOR3D_LENGTH, nSupportVectors);
      rhoBlock = new DenseMatrix64F(nSupportVectors, 1);
      forceMatrix = new DenseMatrix64F(VECTOR3D_LENGTH, 1);

      phiArray = new double[phi.getNumElements()];
      xiArray = new double[xi.getNumElements()];

      leeGoswamiForceOptimizerNative = new LeeGoswamiForceOptimizerNative(nRows, nColumns);

      wrenchError = new SpatialForceVector(centerOfMassFrame);

      parentRegistry.addChild(registry);
   }

   public void setWk(double wk)
   {
      this.wk.set(wk);
   }

   public void setEpsilonF(double epsilonF)
   {
      this.epsilonF.set(epsilonF);
   }

   public void solve(LinkedHashMap<PlaneContactState, FrameVector> forces, HashMap<PlaneContactState, Double> coefficientsOfFriction,
                     SpatialForceVector desiredNetSpatialForceVector)
   {
      int startColumn = 0;
      for (PlaneContactState contactState : forces.keySet())
      {
         // betaBlock
         double mu = coefficientsOfFriction.get(contactState);
         DenseMatrix64F betaBlock = computeBetaBlock(contactState, mu, nSupportVectors, centerOfMassFrame);

         // deltaBlock
         DenseMatrix64F deltaBlock = computeDeltaBlock(contactState, betaBlock, centerOfMassFrame, wk.getDoubleValue());

         // update phi
         CommonOps.insert(deltaBlock, phi, 0, startColumn);
         CommonOps.insert(betaBlock, phi, deltaBlock.getNumRows(), startColumn);
         startColumn += betaBlock.getNumCols();
      }

      MatrixTools.denseMatrixToArrayColumnMajor(phi, 0, 0, phi.getNumRows(), phi.getNumCols(), phiArray, 0);

      desiredNetSpatialForceVector.changeFrame(centerOfMassFrame);
      // kd
      Vector3d desiredNetTorque = new Vector3d();
      desiredNetSpatialForceVector.packAngularPart(desiredNetTorque);
      desiredNetTorque.scale(wk.getDoubleValue());
      MatrixTools.setDenseMatrixFromTuple3d(xi, desiredNetTorque, 0, 0);

      // ld - mg
      Vector3d desiredNetForce = new Vector3d();
      desiredNetSpatialForceVector.packLinearPart(desiredNetForce);
      MatrixTools.setDenseMatrixFromTuple3d(xi, desiredNetForce, VECTOR3D_LENGTH, 0);

      // xi
      MatrixTools.denseMatrixToArrayColumnMajor(xi, 0, 0, xi.getNumRows(), xi.getNumCols(), xiArray, 0);

      double epsilonF = this.epsilonF.getDoubleValue();

      try
      {
         leeGoswamiForceOptimizerNative.solve(phiArray, xiArray, epsilonF);
      }
      catch (NoConvergenceException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      double[] rhoArray = leeGoswamiForceOptimizerNative.getRho();
      rho.set(rho.getNumRows(), rho.getNumCols(), false, rhoArray);

      wrenchError.set(desiredNetSpatialForceVector);
      int i = 0;
      FramePoint ankle = new FramePoint(ReferenceFrame.getWorldFrame());
      FrameVector torque = new FrameVector(ReferenceFrame.getWorldFrame());
      for (PlaneContactState contactState : forces.keySet())
      {
         // betaBlock
         CommonOps.extract(phi, VECTOR3D_LENGTH, VECTOR3D_LENGTH + betaBlock.getNumRows(), i * betaBlock.getNumCols(), (i + 1) * betaBlock.getNumCols(), betaBlock, 0, 0);

         // rhoBlock
         CommonOps.extract(rho, i * rhoBlock.getNumRows(), (i + 1) * rhoBlock.getNumRows(), 0, 1, rhoBlock, 0, 0);

         // force
         CommonOps.mult(betaBlock, rhoBlock, forceMatrix);

         FrameVector force = forces.get(contactState);
         force.setToZero(centerOfMassFrame);
         MatrixTools.denseMatrixToVector3d(forceMatrix, force.getVector(), 0, 0);

         ankle.setToZero(contactState.getBodyFrame());
         ankle.changeFrame(force.getReferenceFrame());

         torque.setToZero(force.getReferenceFrame());
         torque.cross(ankle, force);

         force.checkReferenceFrameMatch(wrenchError.getExpressedInFrame());
         torque.checkReferenceFrameMatch(wrenchError.getExpressedInFrame());

         wrenchError.subAngularPart(torque.getVector());
         wrenchError.subLinearPart(force.getVector());
         i++;
      }
   }

   public Vector3d getTorqueError()
   {
      return wrenchError.getAngularPartCopy();
   }

   // TODO: garbage
   private static DenseMatrix64F computeBetaBlock(PlaneContactState contactState, double mu, int nSupportVectors, ReferenceFrame centerOfMassFrame)
   {
      DenseMatrix64F betaBlock = new DenseMatrix64F(VECTOR3D_LENGTH, nSupportVectors);
      double angleIncrement = 2.0 * Math.PI / nSupportVectors;
      for (int i = 0; i < nSupportVectors; i++)
      {
         double angle = i * angleIncrement;
         double x = mu * Math.cos(angle);
         double y = mu * Math.sin(angle);
         double z = 1.0;
         FrameVector supportVector = new FrameVector(contactState.getPlaneFrame(), x, y, z);
         supportVector.changeFrame(centerOfMassFrame);

         MatrixTools.setDenseMatrixFromTuple3d(betaBlock, supportVector.getVector(), 0, i);
      }

      return betaBlock;
   }

   // TODO: garbage
   private static DenseMatrix64F computeDeltaBlock(PlaneContactState contactState, DenseMatrix64F betaBlock, ReferenceFrame centerOfMassFrame, double wk)
   {
      FramePoint ankle = new FramePoint(contactState.getBodyFrame());
      ankle.changeFrame(centerOfMassFrame);

      DenseMatrix64F skew = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      MatrixTools.vectorToSkewSymmetricMatrix(skew, ankle.getPoint());

      DenseMatrix64F deltaBlock = new DenseMatrix64F(VECTOR3D_LENGTH, betaBlock.getNumCols());
      CommonOps.mult(skew, betaBlock, deltaBlock);

      CommonOps.scale(wk, deltaBlock);

      return deltaBlock;
   }
}

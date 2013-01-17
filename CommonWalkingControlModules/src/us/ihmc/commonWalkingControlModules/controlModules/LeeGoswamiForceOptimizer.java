package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.HashMap;
import java.util.LinkedHashMap;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiForceOptimizer
{
   private static final int VECTOR3D_LENGTH = 3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable wk = new DoubleYoVariable("wk", registry);    // TODO: better name
   private final DoubleYoVariable wf = new DoubleYoVariable("wf", registry);    // TODO: better name
   private final DoubleYoVariable epsilonF = new DoubleYoVariable("epsilonF", registry);    // TODO: better name

   private final ReferenceFrame centerOfMassFrame;
   private final FrameVector gravitationalForce;
   private final int nSupportVectors;

   private final DenseMatrix64F phi = new DenseMatrix64F(0);    // note: this is phi as in the paper *without* the bottom epsilonF * 1 block
   private final DenseMatrix64F beta = new DenseMatrix64F(0);
   private final DenseMatrix64F delta = new DenseMatrix64F(0);
   private final DenseMatrix64F xi = new DenseMatrix64F(0);    // note this is xi as in the paper *without* the zeros appended at the end

   public LeeGoswamiForceOptimizer(ReferenceFrame centerOfMassFrame, FrameVector gravitationalAcceleration, double mass, int nSupportVectors,
                                   YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      this.gravitationalForce = new FrameVector(gravitationalAcceleration);
      gravitationalForce.scale(mass);
      gravitationalForce.changeFrame(centerOfMassFrame);

      this.nSupportVectors = nSupportVectors;

      parentRegistry.addChild(registry);
   }

   public void reshape(int nContacts)
   {
      phi.reshape(Momentum.SIZE, nContacts * nSupportVectors);
      beta.reshape(VECTOR3D_LENGTH, nContacts * nSupportVectors);
      delta.reshape(VECTOR3D_LENGTH, nContacts * nSupportVectors);
      xi.reshape(Momentum.SIZE, 1);
   }

   public void solve(LinkedHashMap<PlaneContactState, FrameVector> forces, HashMap<PlaneContactState, Double> coefficientsOfFriction,
                     SpatialForceVector desiredNetSpatialForceVector)
   {
      reshape(forces.size());

      // phi
      int startColumn = 0;
      for (PlaneContactState contactState : forces.keySet())
      {
         double mu = coefficientsOfFriction.get(contactState);

         // update beta
         DenseMatrix64F betaBlock = computeBetaBlock(contactState, mu, nSupportVectors, centerOfMassFrame);
         CommonOps.insert(betaBlock, beta, 0, startColumn);

         // update delta
         DenseMatrix64F deltaBlock = computeDeltaBlock(contactState, betaBlock, centerOfMassFrame);
         CommonOps.insert(deltaBlock, delta, 0, startColumn);

         startColumn += betaBlock.getNumCols();
      }

      int startRow = 0;
      CommonOps.insert(beta, phi, startRow, 0);
      startRow += beta.getNumRows();

      MatrixTools.setMatrixBlock(delta, 0, 0, phi, startRow, 0, delta.getNumRows(), delta.getNumCols(), wk.getDoubleValue());
      startRow += delta.getNumRows();

      // xi
      Vector3d desiredNetForce = new Vector3d();
      desiredNetSpatialForceVector.packLinearPart(desiredNetForce);
      desiredNetForce.sub(gravitationalForce.getVector());
      MatrixTools.setDenseMatrixFromTuple3d(xi, desiredNetForce, 0, 0);

      Vector3d desiredNetTorque = new Vector3d();
      desiredNetSpatialForceVector.packAngularPart(desiredNetTorque);
      desiredNetTorque.scale(wf.getDoubleValue());
      MatrixTools.setDenseMatrixFromTuple3d(xi, desiredNetTorque, VECTOR3D_LENGTH, 0);

      double epsilonF = this.epsilonF.getDoubleValue();

      // TODO: call force solver with phi, xi, and epsilonf
      // TODO: set forces (expressed in CoM frame)
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
   private static DenseMatrix64F computeDeltaBlock(PlaneContactState contactState, DenseMatrix64F betaBlock, ReferenceFrame centerOfMassFrame)
   {
      FramePoint ankle = new FramePoint(contactState.getBodyFrame());
      ankle.changeFrame(centerOfMassFrame);

      DenseMatrix64F skew = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      MatrixTools.vectorToSkewSymmetricMatrix(skew, ankle.getPoint());

      DenseMatrix64F deltaBlock = new DenseMatrix64F(VECTOR3D_LENGTH, betaBlock.getNumCols());
      CommonOps.mult(skew, betaBlock, deltaBlock);

      return deltaBlock;
   }
}

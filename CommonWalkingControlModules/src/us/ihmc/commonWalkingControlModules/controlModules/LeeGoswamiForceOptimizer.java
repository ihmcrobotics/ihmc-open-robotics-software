package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
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
   private final DoubleYoVariable wf = new DoubleYoVariable("wf", registry);    // TODO: better name
   private final DoubleYoVariable epsilonF = new DoubleYoVariable("epsilonF", registry);    // TODO: better name

   private final ReferenceFrame centerOfMassFrame;
   private final FrameVector gravitationalForce;
   private final int nSupportVectors;

   // column major:
   private final double[] phi; // note: this is phi as in the paper *without* the bottom epsilonF * 1 block
   private final double[] xi;    // note: this is xi as in the paper *without* the zeros appended at the end

   private final LeeGoswamiForceOptimizerNative leeGoswamiForceOptimizerNative;
   
   public LeeGoswamiForceOptimizer(ReferenceFrame centerOfMassFrame, FrameVector gravitationalAcceleration, double mass, int nSupportVectors,
                                   YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      this.gravitationalForce = new FrameVector(gravitationalAcceleration);
      gravitationalForce.scale(mass);
      gravitationalForce.changeFrame(centerOfMassFrame);

      this.nSupportVectors = nSupportVectors;

      int maxNContacts = 2;
      int nRows = 2 * VECTOR3D_LENGTH;
      int nColumns = maxNContacts * nSupportVectors;
      phi = new double[nRows * nColumns];
      xi = new double[nRows];
      
      leeGoswamiForceOptimizerNative = new LeeGoswamiForceOptimizerNative(nRows, nColumns);

      parentRegistry.addChild(registry);
   }

   public void solve(LinkedHashMap<PlaneContactState, FrameVector> forces, HashMap<PlaneContactState, Double> coefficientsOfFriction,
                     SpatialForceVector desiredNetSpatialForceVector)
   {
      // phi
      DenseMatrix64F phiBlock = new DenseMatrix64F(2 * VECTOR3D_LENGTH, nSupportVectors);
      int startIndex = 0;
      Arrays.fill(phi, 0.0);
      for (PlaneContactState contactState : forces.keySet())
      {
         double mu = coefficientsOfFriction.get(contactState);

         // update beta
         DenseMatrix64F betaBlock = computeBetaBlock(contactState, mu, nSupportVectors, centerOfMassFrame);
         CommonOps.insert(betaBlock, phiBlock, 0, 0);

         // update delta
         DenseMatrix64F deltaBlock = computeDeltaBlock(contactState, betaBlock, centerOfMassFrame);
         CommonOps.scale(wf.getDoubleValue(), deltaBlock);
         CommonOps.insert(deltaBlock, phiBlock, VECTOR3D_LENGTH, 0);

         MatrixTools.denseMatrixToArrayColumnMajor(phiBlock, 0, 0, phiBlock.getNumRows(), phiBlock.getNumCols(), phi, startIndex);
         startIndex += phiBlock.getNumElements();
      }

      // xi
      Arrays.fill(xi, 0.0);
      Vector3d desiredNetForce = new Vector3d();
      desiredNetSpatialForceVector.packLinearPart(desiredNetForce);
      desiredNetForce.sub(gravitationalForce.getVector());
      MatrixTools.tuple3dToArray(desiredNetForce, xi, 0);

      Vector3d desiredNetTorque = new Vector3d();
      desiredNetSpatialForceVector.packAngularPart(desiredNetTorque);
      desiredNetTorque.scale(wf.getDoubleValue());
      MatrixTools.tuple3dToArray(desiredNetTorque, xi, VECTOR3D_LENGTH);

      double epsilonF = this.epsilonF.getDoubleValue();

      try
      {
         leeGoswamiForceOptimizerNative.solve(phi, xi, epsilonF);
      }
      catch (NoConvergenceException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      
      double[] forceArray = leeGoswamiForceOptimizerNative.getRho(); 
      

      int index = 0;
      for (FrameVector force : forces.values())
      {
         // FIXME: what I get back is rho, not f
         force.setToZero(centerOfMassFrame);
         force.setX(forceArray[index++]);
         force.setY(forceArray[index++]);
         force.setZ(forceArray[index++]);
      }
   }

   public Vector3d getTorqueError()
   {
      // FIXME
      throw new RuntimeException("not yet implemented");
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

package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class LeeGoswamiGroundReactionWrenchDistributor
{
   private static final int VECTOR3D_LENGTH = 3;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable wk = new DoubleYoVariable("wk", registry);    // TODO: better name
   private final DoubleYoVariable wf = new DoubleYoVariable("wf", registry);    // TODO: better name
   private final DoubleYoVariable epsilonF = new DoubleYoVariable("epsilonF", registry);    // TODO: better name
   private final DoubleYoVariable epsilonP = new DoubleYoVariable("epsilonP", registry);    // TODO: better name

   private final ReferenceFrame centerOfMassFrame;

   private final List<PlaneContactState> contactStates = new ArrayList<PlaneContactState>();
   private final HashMap<PlaneContactState, Double> coefficientsOfFriction = new HashMap<PlaneContactState, Double>();
   private final HashMap<PlaneContactState, Double> rotationalCoefficientsOfFriction = new HashMap<PlaneContactState, Double>();

   private final HashMap<PlaneContactState, FrameVector> forces = new HashMap<PlaneContactState, FrameVector>();
   private final HashMap<PlaneContactState, FramePoint2d> centersOfPressure = new HashMap<PlaneContactState, FramePoint2d>();
   private final HashMap<PlaneContactState, Double> normalTorques = new HashMap<PlaneContactState, Double>();

   // TODO: pass into constructor:
   private final FrameVector gravitationalForce;
   private final int nSupportVectors;

   private final DenseMatrix64F phi = new DenseMatrix64F(0);    // note: this is phi as in the paper *without* the bottom epsilonF * 1 block
   private final DenseMatrix64F beta = new DenseMatrix64F(0);
   private final DenseMatrix64F delta = new DenseMatrix64F(0);
   private final DenseMatrix64F xi = new DenseMatrix64F(0);    // note this is xi as in the paper *without* the zeros appended at the end

   private final DenseMatrix64F psik = new DenseMatrix64F(0);
   private final DenseMatrix64F kappaK = new DenseMatrix64F(0);
   private final DenseMatrix64F etaMin = new DenseMatrix64F(0);
   private final DenseMatrix64F etaMax = new DenseMatrix64F(0);
   private final DenseMatrix64F etaD = new DenseMatrix64F(0);

   private final Matrix3d tempMatrix = new Matrix3d();


   public LeeGoswamiGroundReactionWrenchDistributor(ReferenceFrame centerOfMassFrame, FrameVector gravitationalAcceleration, double mass, int nSupportVectors,
           YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;

      this.gravitationalForce = new FrameVector(gravitationalAcceleration);
      gravitationalForce.scale(mass);
      gravitationalForce.changeFrame(centerOfMassFrame);

      this.nSupportVectors = nSupportVectors;

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      // TODO: keep a storage of setting and result objects that persists and can be coupled to different contact states 
      contactStates.clear();
      coefficientsOfFriction.clear();
      rotationalCoefficientsOfFriction.clear();
      forces.clear();
      centersOfPressure.clear();
      normalTorques.clear();
   }

   public void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFriction)
   {
      contactStates.add(contactState);
      coefficientsOfFriction.put(contactState, coefficientOfFriction);
      rotationalCoefficientsOfFriction.put(contactState, rotationalCoefficientOfFriction);
      forces.put(contactState, new FrameVector(contactState.getBodyFrame()));
      centersOfPressure.put(contactState, new FramePoint2d(contactState.getPlaneFrame()));
      normalTorques.put(contactState, 0.0);

      int nContacts = contactStates.size();
      phi.reshape(Momentum.SIZE, nContacts * nSupportVectors);
      beta.reshape(VECTOR3D_LENGTH, nContacts * nSupportVectors);
      delta.reshape(VECTOR3D_LENGTH, nContacts * nSupportVectors);
      xi.reshape(Momentum.SIZE, 1);

      psik.reshape(VECTOR3D_LENGTH, nContacts * VECTOR3D_LENGTH);
      kappaK.reshape(VECTOR3D_LENGTH, 1);
      etaMin.reshape(nContacts * VECTOR3D_LENGTH, 1);
      etaMax.reshape(nContacts * VECTOR3D_LENGTH, 1);
      etaD.reshape(nContacts * VECTOR3D_LENGTH, 1);
   }

   public void solve(SpatialForceVector desiredNetSpatialForceVector)
   {
      desiredNetSpatialForceVector.changeFrame(centerOfMassFrame);

      solveForces(forces, desiredNetSpatialForceVector);
      solveCoPsAndNormalTorques(centersOfPressure, normalTorques, desiredNetSpatialForceVector.getAngularPartCopy(), forces);
   }

   private void solveForces(HashMap<PlaneContactState, FrameVector> forces, SpatialForceVector desiredNetSpatialForceVector)
   {
      // phi
      int startColumn = 0;
      for (PlaneContactState contactState : contactStates)
      {
         double mu = coefficientsOfFriction.get(contactState);

         // update beta
         DenseMatrix64F betaBlock = computeBetaBlock(contactState, mu, nSupportVectors, centerOfMassFrame);
         CommonOps.insert(betaBlock, beta, 0, startColumn);

         // update delta TODO
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

   // TODO: assumes that ankle is directly above origin of planeFrame for a contact state
   private void solveCoPsAndNormalTorques(HashMap<PlaneContactState, FramePoint2d> centersOfPressure, HashMap<PlaneContactState, Double> normalTorques,
           Vector3d desiredNetTorque, HashMap<PlaneContactState, FrameVector> forces)
   {
      // psiK, kappaK
      MatrixTools.setDenseMatrixFromTuple3d(kappaK, desiredNetTorque, 0, 0);
      int startColumn = 0;
      
      // TODO: garbage
      DenseMatrix64F a = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      DenseMatrix64F skew = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
      FramePoint ankle = new FramePoint(ReferenceFrame.getWorldFrame());

      for (PlaneContactState contactState : contactStates)
      {
         FrameVector force = forces.get(contactState);
         force.changeFrame(centerOfMassFrame);

         MatrixTools.vectorToSkewSymmetricMatrix(skew, force.getVector());

         Transform3D transform = contactState.getBodyFrame().getTransformToDesiredFrame(centerOfMassFrame);
         transform.get(tempMatrix);
         DenseMatrix64F rotationMatrix = new DenseMatrix64F(VECTOR3D_LENGTH, VECTOR3D_LENGTH);
         MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, rotationMatrix);

         CommonOps.mult(-1.0, skew, rotationMatrix, a);

         // update psiK
         CommonOps.extract(a, 0, a.getNumRows(), 0, 2, psik, 0, startColumn); // first two columns of A
         CommonOps.extract(rotationMatrix, 0, rotationMatrix.getNumRows(), 2, 3, psik, 0, startColumn + 2); // last column of R
         startColumn += VECTOR3D_LENGTH;

         // update kappaK
         ankle.setToZero(contactState.getBodyFrame());
         ankle.changeFrame(contactState.getPlaneFrame());
         double ankleHeight = ankle.getZ();
         MatrixTools.addMatrixBlock(kappaK, 0, 0, a, 0, 2, a.getNumRows(), 1, ankleHeight); // last column of A times h
      }

      // etaMin, etaMax (assumes rectangular feet)
      int startRow = 0;
      for (PlaneContactState contactState : contactStates)
      {
         FrameVector force = forces.get(contactState);
         force.changeFrame(contactState.getPlaneFrame());
         double normalForce = force.getZ();
         double tauMax = rotationalCoefficientsOfFriction.get(contactState) * normalForce;

         List<FramePoint2d> contactPoints = contactState.getContactPoints2d();
         double xMin = Double.POSITIVE_INFINITY;
         double xMax = Double.NEGATIVE_INFINITY;
         double yMin = Double.POSITIVE_INFINITY;
         double yMax = Double.NEGATIVE_INFINITY;

         for (FramePoint2d contactPoint : contactPoints)
         {
            contactPoint.changeFrame(contactState.getPlaneFrame());
            double x = contactPoint.getX();
            double y = contactPoint.getY();
            if (x < xMin)
               xMin = x;
            if (x > xMax)
               xMax = x;
            if (y < yMin)
               yMin = y;
            if (y > yMax)
               yMax = y;
         }

         etaMin.set(startRow + 0, 0, xMin);
         etaMin.set(startRow + 1, 0, yMin);
         etaMin.set(startRow + 2, 0, -tauMax);

         etaMax.set(startRow + 0, 0, xMax);
         etaMax.set(startRow + 1, 0, yMax);
         etaMax.set(startRow + 2, 0, tauMax);

         startRow += VECTOR3D_LENGTH;
      }

      // etad (average of min and max for now; could change later
      CommonOps.add(0.5, etaMin, etaMax, etaD);

      double epsilonP = this.epsilonP.getDoubleValue();

      // TODO: call CoP solver with Psi, kappa, etaMin, etaMax, etad, and epsilonP
   }

   public FramePoint2d getCenterOfPressure(PlaneContactState contactState)
   {
      return centersOfPressure.get(contactState);
   }

   public double getNormalTorque(PlaneContactState contactState)
   {
      return normalTorques.get(contactState);
   }

   public FrameVector getForce(PlaneContactState contactState)
   {
      return forces.get(contactState);
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

package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateTermCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;

/**
 * @author twan Date: 5/1/13
 */
public class CentroidalMomentumHandler
{
   private final DenseMatrix64F adotV = new DenseMatrix64F(SpatialMotionVector.SIZE, 1);
   private final DenseMatrix64F centroidalMomentumMatrixPart = new DenseMatrix64F(1, 1);
   private final RigidBody rootBody;
   private final RigidBody[] rigidBodyList;
   private final SpatialForceVector centroidalMomentumRate;

   private final InverseDynamicsJoint[] jointsInOrder;
   private final DenseMatrix64F v;
   private final Map<InverseDynamicsJoint, int[]> columnsForJoints = new LinkedHashMap<InverseDynamicsJoint, int[]>();
   private final DenseMatrix64F spatialMomentum = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F momentumRate = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F centroidalMomentumEquationRightHandSide = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final ReferenceFrame centerOfMassFrame;
   private final CentroidalMomentumRateTermCalculator centroidalMomentumRateTermCalculator;
   private final double robotMass;
   /**
    * Using nomenclature from "Centroidal dynamics of a humanoid robot", by David E. Orin, Ambarish Goswami, Sung-Hee Lee
    */
   private final DenseMatrix64F centroidalCompositeRigidBodyInertia = new DenseMatrix64F(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
   private final FrameVector3D centroidalLinearMomentum;
   private final FrameVector3D centroidalAngularMomentum;
   private final FrameVector3D centroidalLinearVelocity;
   private final FrameVector3D centroidalAngularVelocity;
   private final int angularPartSize = SpatialForceVector.SIZE / 2;
   private final DenseMatrix64F centroidalAngularInertia = new DenseMatrix64F(angularPartSize, angularPartSize);
   private final RigidBodyInertia tempRigidBodyInertia = new RigidBodyInertia(ReferenceFrame.getWorldFrame(), 0.01, 0.01, 0.01, 0.01); // Dummy value to create a valid inertia tensor
   private final DenseMatrix64F tempMatrixForRigidBodyInertia = new DenseMatrix64F(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
   private final LinearSolver<DenseMatrix64F> linear3DMatrixSolver = LinearSolverFactory.symmPosDef(angularPartSize);
   private final DenseMatrix64F A = new DenseMatrix64F(angularPartSize, angularPartSize), b = new DenseMatrix64F(angularPartSize, 1),
         x = new DenseMatrix64F(angularPartSize, 1);

   public CentroidalMomentumHandler(RigidBody rootBody, ReferenceFrame centerOfMassFrame)
   {
      this.rootBody = rootBody;
      this.jointsInOrder = ScrewTools.computeSupportAndSubtreeJoints(rootBody);
      if (rootBody.getChildrenJoints().size() != 1)
         throw new RuntimeException("More than one root joint");
      this.rigidBodyList = ScrewTools.computeRigidBodiesAfterThisJoint(rootBody.getChildrenJoints().get(0));

      int nDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(jointsInOrder);
      this.v = new DenseMatrix64F(nDegreesOfFreedom, 1);

      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         TIntArrayList listToPackIndices = new TIntArrayList();
         ScrewTools.computeIndexForJoint(jointsInOrder, listToPackIndices, joint);
         int[] indices = listToPackIndices.toArray();
         columnsForJoints.put(joint, indices);
      }

      centroidalMomentumRate = new SpatialForceVector(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;

      robotMass = TotalMassCalculator.computeSubTreeMass(rootBody);
      this.centroidalMomentumRateTermCalculator = new CentroidalMomentumRateTermCalculator(rootBody, centerOfMassFrame, v, robotMass);
      this.centroidalLinearMomentum = new FrameVector3D(centerOfMassFrame);
      this.centroidalAngularMomentum = new FrameVector3D(centerOfMassFrame);
      this.centroidalLinearVelocity = new FrameVector3D(centerOfMassFrame);
      this.centroidalAngularVelocity = new FrameVector3D(centerOfMassFrame);
   }

   public void initialize()
   {

   }

   public void compute()
   {
      ScrewTools.getJointVelocitiesMatrix(jointsInOrder, v);

      centroidalMomentumRateTermCalculator.compute();
      adotV.set(centroidalMomentumRateTermCalculator.getADotVTerm());
      computeSpatialMomentum();
      computeCentroidalCompositeRigidBodyInertia();
      computeCentroidalSpatialVelocity();
   }

   private void computeSpatialMomentum()
   {
      DenseMatrix64F centroidalMomentumMatrix = centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix();
      CommonOps.mult(centroidalMomentumMatrix, v, spatialMomentum);
      setLinearAndAngularMomentumfromSpatialMomentum();
   }

   private void setLinearAndAngularMomentumfromSpatialMomentum()
   {
      centroidalAngularMomentum.setIncludingFrame(centerOfMassFrame, 0, spatialMomentum);
      centroidalLinearMomentum.setIncludingFrame(centerOfMassFrame, 3, spatialMomentum);
   }

   private void computeCentroidalCompositeRigidBodyInertia()
   {
      tempMatrixForRigidBodyInertia.reshape(SpatialForceVector.SIZE, SpatialForceVector.SIZE);

      centroidalCompositeRigidBodyInertia.reshape(SpatialForceVector.SIZE, SpatialForceVector.SIZE);
      centroidalCompositeRigidBodyInertia.zero();

      for (int i = 0; i < rigidBodyList.length; i++)
      {
         RigidBodyInertia rigidBodyInertia = rigidBodyList[i].getInertia();
         tempRigidBodyInertia.set(rigidBodyInertia);
         tempRigidBodyInertia.changeFrame(centerOfMassFrame);
         tempRigidBodyInertia.getMatrix(tempMatrixForRigidBodyInertia);
         CommonOps.addEquals(centroidalCompositeRigidBodyInertia, tempMatrixForRigidBodyInertia);
      }
      setLinearAndAngularInertia();
   }

   private void setLinearAndAngularInertia()
   {
      //No need to set linear inertia. Its just the robot mass
      CommonOps.extract(centroidalCompositeRigidBodyInertia, 0, 3, 0, 3, centroidalAngularInertia, 0, 0);
   }

   private void computeCentroidalSpatialVelocity()
   {
      // Storing as linear and angular components since its more efficient that way
      centroidalLinearVelocity.setIncludingFrame(centroidalLinearMomentum);
      centroidalLinearVelocity.scale(1.0 / robotMass);

      A.set(centroidalAngularInertia);
      centroidalAngularMomentum.get(b);
      x.reshape(angularPartSize, 1);
      linear3DMatrixSolver.setA(A);
      linear3DMatrixSolver.solve(b, x);
      centroidalAngularVelocity.setIncludingFrame(centerOfMassFrame, x);
   }

   public void getSpatialMomemntum(SpatialForceVector spatialMomentum)
   {
      spatialMomentum.setToZero(centerOfMassFrame);
      spatialMomentum.setAngularPart(centroidalAngularMomentum);
      spatialMomentum.setLinearPart(centroidalLinearMomentum);
   }

   public void getSpatialVelocity(SpatialMotionVector spatialVelocity)
   {
      spatialVelocity.setToZero(centerOfMassFrame, ReferenceFrame.getWorldFrame(), centerOfMassFrame);
      spatialVelocity.setAngularPart(centroidalAngularVelocity);
      spatialVelocity.setLinearPart(centroidalLinearVelocity);
   }

   public void getAngularMomentum(FrameVector3D angularMomentumToPack)
   {
      angularMomentumToPack.setIncludingFrame(centroidalAngularMomentum);
   }

   public void getLinearMomentum(FrameVector3D linearMomentumToPack)
   {
      linearMomentumToPack.setIncludingFrame(centroidalLinearMomentum);
   }

   public void getCentroidalAngularInertia(DenseMatrix64F angularInertiaToPack)
   {
      angularInertiaToPack.set(centroidalAngularInertia);
   }

   public void getCenterOfMassLinearVelocity(FrameVector3D centerOfMassLinearVelocityToPack)
   {
      centerOfMassLinearVelocityToPack.setIncludingFrame(centroidalLinearVelocity);
   }

   public void getCenterOfMassAngularVelocity(FrameVector3D centerOfMassAngularVelocityToPack)
   {
      centerOfMassAngularVelocityToPack.setIncludingFrame(centroidalAngularVelocity);
   }

   public DenseMatrix64F getCentroidalMomentumMatrixPart(InverseDynamicsJoint[] joints)
   {
      int partDegreesOfFreedom = ScrewTools.computeDegreesOfFreedom(joints);
      centroidalMomentumMatrixPart.reshape(Momentum.SIZE, partDegreesOfFreedom);
      centroidalMomentumMatrixPart.zero();
      int startColumn = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int[] columnsForJoint = columnsForJoints.get(joint);
         MatrixTools.extractColumns(centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix(), columnsForJoint, centroidalMomentumMatrixPart,
                                    startColumn);
         startColumn += columnsForJoint.length;
      }
      return centroidalMomentumMatrixPart;
   }

   public DenseMatrix64F getCentroidalMomentumConvectiveTerm()
   {
      return adotV;
   }

   public void computeCentroidalMomentumRate(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations)
   {
      DenseMatrix64F centroidalMomentumMatrixPart = getCentroidalMomentumMatrixPart(jointsToOptimizeFor);
      CommonOps.mult(centroidalMomentumMatrixPart, jointAccelerations, momentumRate);
      CommonOps.addEquals(momentumRate, adotV);
      centroidalMomentumRate.set(centerOfMassFrame, momentumRate);
   }

   public SpatialForceVector getCentroidalMomentumRate()
   {
      return centroidalMomentumRate;
   }

   public DenseMatrix64F getMomentumDotEquationRightHandSide(MomentumRateCommand momentumRateCommand)
   {
      momentumRateCommand.getSelectionMatrix(centerOfMassFrame, selectionMatrix);
      DenseMatrix64F momentumRate = momentumRateCommand.getMomentumRate();

      CommonOps.mult(selectionMatrix, momentumRate, centroidalMomentumEquationRightHandSide);
      CommonOps.subtractEquals(centroidalMomentumEquationRightHandSide, adotV);
      return centroidalMomentumEquationRightHandSide;
   }

   public void computeSpatialInertiaMatrix(DenseMatrix64F spatialCentroidalInertiaToPack)
   {
      spatialCentroidalInertiaToPack.set(centroidalCompositeRigidBodyInertia);
   }
}

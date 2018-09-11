package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateTermCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

/**
 * @author twan Date: 5/1/13
 */
public class CentroidalMomentumHandler
{
   private final SpatialForceVector centroidalMomentumRate;

   private final DenseMatrix64F centroidalMomentumEquationRightHandSide = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final ReferenceFrame centerOfMassFrame;
   private final CentroidalMomentumRateTermCalculator centroidalMomentumRateTermCalculator;

   public CentroidalMomentumHandler(RigidBody rootBody, ReferenceFrame centerOfMassFrame)
   {
      this(ScrewTools.computeSubtreeJoints(rootBody), centerOfMassFrame);
   }

   public CentroidalMomentumHandler(InverseDynamicsJoint[] jointsToConsider, ReferenceFrame centerOfMassFrame)
   {
      centroidalMomentumRate = new SpatialForceVector(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;
      this.centroidalMomentumRateTermCalculator = new CentroidalMomentumRateTermCalculator(jointsToConsider, centerOfMassFrame);
   }

   public void initialize()
   {
   }

   public void compute()
   {
      centroidalMomentumRateTermCalculator.reset();
   }

   public void getAngularMomentum(FrameVector3D angularMomentumToPack)
   {
      centroidalMomentumRateTermCalculator.getMomentum().getAngularPartIncludingFrame(angularMomentumToPack);
   }

   public void getLinearMomentum(FrameVector3D linearMomentumToPack)
   {
      centroidalMomentumRateTermCalculator.getMomentum().getLinearPartIncludingFrame(linearMomentumToPack);
   }

   public void getCenterOfMassVelocity(FrameVector3D centerOfMassVelocityToPack)
   {
      centerOfMassVelocityToPack.setIncludingFrame(centroidalMomentumRateTermCalculator.getCenterOfMassVelocity());
   }

   public DenseMatrix64F getCentroidalMomentumMatrixPart()
   {
      return centroidalMomentumRateTermCalculator.getCentroidalMomentumMatrix();
   }

   public DenseMatrix64F getCentroidalMomentumConvectiveTerm()
   {
      return centroidalMomentumRateTermCalculator.getBiasSpatialForceMatrix();
   }

   public void computeCentroidalMomentumRate(DenseMatrix64F jointAccelerations)
   {
      centroidalMomentumRateTermCalculator.getMomentumRate(jointAccelerations, centroidalMomentumRate);
   }

   public SpatialForceVector getCentroidalMomentumRate()
   {
      return centroidalMomentumRate;
   }

   public DenseMatrix64F getMomentumDotEquationRightHandSide(MomentumRateCommand momentumRateCommand)
   {
      momentumRateCommand.getSelectionMatrix(centerOfMassFrame, selectionMatrix);
      DenseMatrix64F momentumRate = momentumRateCommand.getMomentumRate();

      DenseMatrix64F biasSpatialForceMatrix = centroidalMomentumRateTermCalculator.getBiasSpatialForceMatrix();
      CommonOps.mult(selectionMatrix, momentumRate, centroidalMomentumEquationRightHandSide);
      CommonOps.subtractEquals(centroidalMomentumEquationRightHandSide, biasSpatialForceMatrix);
      return centroidalMomentumEquationRightHandSide;
   }
}

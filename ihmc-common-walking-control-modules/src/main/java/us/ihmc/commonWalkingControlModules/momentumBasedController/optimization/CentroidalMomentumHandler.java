package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;
import us.ihmc.robotics.screwTheory.CentroidalMomentumRateTermCalculator;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

/**
 * @author twan Date: 5/1/13
 */
public class CentroidalMomentumHandler
{
   private final SpatialForce centroidalMomentumRate;
   private final Momentum centroidalMomentum;

   private final DenseMatrix64F centroidalMomentumEquationRightHandSide = new DenseMatrix64F(Momentum.SIZE, 1);
   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private final ReferenceFrame centerOfMassFrame;
   private final CentroidalMomentumRateTermCalculator centroidalMomentumRateTermCalculator;

   public CentroidalMomentumHandler(RigidBody rootBody, ReferenceFrame centerOfMassFrame)
   {
      this(ScrewTools.computeSubtreeJoints(rootBody), centerOfMassFrame);
   }

   public CentroidalMomentumHandler(JointBasics[] jointsToConsider, ReferenceFrame centerOfMassFrame)
   {
      centroidalMomentumRate = new SpatialForce(centerOfMassFrame);
      centroidalMomentum = new Momentum(centerOfMassFrame);
      this.centerOfMassFrame = centerOfMassFrame;
      this.centroidalMomentumRateTermCalculator = new CentroidalMomentumRateTermCalculator(jointsToConsider, centerOfMassFrame);
   }

   public void initialize()
   {
   }

   /**
    * Invalidates the internal data such that it will be updated when accessing any information such as the momentum or the centroidal momentum matrix.
    * @deprecated Use {@link #reset()} instead
    */
   public void compute()
   {
      reset();
   }

   /**
    * Invalidates the internal data such that it will be updated when accessing any information such as the momentum or the centroidal momentum matrix.
    */
   public void reset()
   {
      centroidalMomentumRateTermCalculator.reset();
   }

   public void getAngularMomentum(FrameVector3D angularMomentumToPack)
   {
      angularMomentumToPack.setIncludingFrame(centroidalMomentumRateTermCalculator.getMomentum().getAngularPart());
   }

   public void getLinearMomentum(FrameVector3D linearMomentumToPack)
   {
      linearMomentumToPack.setIncludingFrame(centroidalMomentumRateTermCalculator.getMomentum().getLinearPart());
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

   public void computeCentroidalMomentum(DenseMatrix64F jointVelocities)
   {
      centroidalMomentumRateTermCalculator.getMomentum(jointVelocities, centroidalMomentum);
   }

   public SpatialForceReadOnly getCentroidalMomentumRate()
   {
      return centroidalMomentumRate;
   }

   public MomentumReadOnly getCentroidalMomentum()
   {
      return centroidalMomentum;
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

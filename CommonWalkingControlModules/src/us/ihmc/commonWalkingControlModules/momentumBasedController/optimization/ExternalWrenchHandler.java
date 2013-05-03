package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

/**
 * @author twan
 *         Date: 5/2/13
 */
public class ExternalWrenchHandler
{
   private final SpatialForceVector gravitationalWrench;
   private final DenseMatrix64F wrenchEquationRightHandSide = new DenseMatrix64F(Wrench.SIZE, 1);

   public ExternalWrenchHandler(double gravityZ, ReferenceFrame centerOfMassFrame, InverseDynamicsJoint rootJoint)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame);
      double totalMass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));
      gravitationalWrench.setLinearPartZ(-gravityZ * totalMass);
   }

   /**
    * Computes \dot{A} * v - external wrenches already applied (e.g. gravity, hand wrenches due to manipulation)
    * @param momentumEquationConvectiveTerm
    * @return
    */
   public final DenseMatrix64F computeWrenchEquationRightHandSide(DenseMatrix64F momentumEquationConvectiveTerm)
   {
      gravitationalWrench.packMatrix(wrenchEquationRightHandSide);
      CommonOps.changeSign(wrenchEquationRightHandSide);
      CommonOps.addEquals(wrenchEquationRightHandSide, momentumEquationConvectiveTerm);
      return wrenchEquationRightHandSide;
   }
}

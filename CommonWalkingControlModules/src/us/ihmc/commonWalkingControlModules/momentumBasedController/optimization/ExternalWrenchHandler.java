package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * @author twan
 *         Date: 5/2/13
 */
public class ExternalWrenchHandler
{
   private final SpatialForceVector gravitationalWrench;
   private final DenseMatrix64F wrenchEquationRightHandSide = new DenseMatrix64F(Wrench.SIZE, 1);
   private final Map<RigidBody, Wrench> externalWrenchesToCompensateFor = new LinkedHashMap<RigidBody, Wrench>();
   private final SpatialForceVector totalWrenchAlreadyApplied; // gravity plus external wrenches to compensate for
   private final SpatialForceVector tempWrench = new SpatialForceVector();

   public ExternalWrenchHandler(double gravityZ, ReferenceFrame centerOfMassFrame, InverseDynamicsJoint rootJoint)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame);
      double totalMass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));
      gravitationalWrench.setLinearPartZ(-gravityZ * totalMass);
      totalWrenchAlreadyApplied = new SpatialForceVector(centerOfMassFrame);
   }

   public void reset()
   {
      externalWrenchesToCompensateFor.clear();
   }

   /**
    * Computes \dot{A} * v - external wrenches already applied (e.g. gravity, hand wrenches due to manipulation)
    * @param momentumEquationConvectiveTerm
    * @return
    */
   public final DenseMatrix64F computeWrenchEquationRightHandSide(DenseMatrix64F momentumEquationConvectiveTerm)
   {
      totalWrenchAlreadyApplied.set(gravitationalWrench);
      for (Wrench externalWrenchToCompensateFor : externalWrenchesToCompensateFor.values())
      {
         tempWrench.set(externalWrenchToCompensateFor);
         tempWrench.changeFrame(gravitationalWrench.getExpressedInFrame());
         totalWrenchAlreadyApplied.add(tempWrench);
      }

      totalWrenchAlreadyApplied.packMatrix(wrenchEquationRightHandSide);
      CommonOps.changeSign(wrenchEquationRightHandSide);
      CommonOps.addEquals(wrenchEquationRightHandSide, momentumEquationConvectiveTerm);
      return wrenchEquationRightHandSide;
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      externalWrenchesToCompensateFor.put(rigidBody, wrench);
   }

   public Map<RigidBody, Wrench> getExternalWrenchesToCompensateFor()
   {
      return externalWrenchesToCompensateFor;
   }
}

package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

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
   private final Map<RigidBody, Wrench> externalWrenches = new LinkedHashMap<RigidBody, Wrench>();
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
    *
    * @return
    */
   public final DenseMatrix64F computeWrenchEquationRightHandSide(DenseMatrix64F momentumConvectiveTerm, DenseMatrix64F b, DenseMatrix64F bHat)
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
      CommonOps.addEquals(wrenchEquationRightHandSide, momentumConvectiveTerm);
      CommonOps.addEquals(wrenchEquationRightHandSide, b);
      CommonOps.subtractEquals(wrenchEquationRightHandSide, bHat);
      return wrenchEquationRightHandSide;
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
      wrench.getBodyFrame().checkReferenceFrameMatch(bodyFixedFrame);
      wrench.getExpressedInFrame().checkReferenceFrameMatch(bodyFixedFrame);
      externalWrenchesToCompensateFor.put(rigidBody, wrench);
   }

   public void computeExternalWrenches(Map<RigidBody, Wrench> groundReactionWrenches)
   {
      this.externalWrenches.clear();

      ScrewTools.addExternalWrenches(externalWrenches, groundReactionWrenches);
      ScrewTools.addExternalWrenches(externalWrenches, externalWrenchesToCompensateFor);
   }

   public Map<RigidBody, Wrench> getExternalWrenches()
   {
      return externalWrenches;
   }
}

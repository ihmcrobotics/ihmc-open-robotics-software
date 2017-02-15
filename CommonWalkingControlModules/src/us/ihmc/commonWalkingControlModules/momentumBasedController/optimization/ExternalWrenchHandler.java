package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 5/2/13
 */
public class ExternalWrenchHandler
{
   private final SpatialForceVector gravitationalWrench;
   private final DenseMatrix64F gravitationalWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final DenseMatrix64F wrenchEquationRightHandSide = new DenseMatrix64F(Wrench.SIZE, 1);
   private final Map<RigidBody, Wrench> externalWrenchesToCompensateFor = new LinkedHashMap<RigidBody, Wrench>();
   /** For garbage free iteration */
   private final List<Wrench> externalWrenchesToCompensateForList = new ArrayList<Wrench>();
   private final SpatialForceVector totalWrenchAlreadyApplied; // gravity plus external wrenches to compensate for
   private final DenseMatrix64F totalWrenchAlreadyAppliedMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final List<RigidBody> rigidBodiesWithWrenchToCompensateFor = new ArrayList<>();
   private final List<RigidBody> rigidBodiesWithExternalWrench = new ArrayList<>();
   private final Map<RigidBody, Wrench> externalWrenches = new LinkedHashMap<RigidBody, Wrench>();
   private final SpatialForceVector tempWrench = new SpatialForceVector();
   private final ReferenceFrame centerOfMassFrame;

   public ExternalWrenchHandler(double gravityZ, ReferenceFrame centerOfMassFrame, InverseDynamicsJoint rootJoint,
         List<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.contactablePlaneBodies = new ArrayList<>(contactablePlaneBodies);

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame);
      double totalMass = TotalMassCalculator.computeMass(ScrewTools.computeSupportAndSubtreeSuccessors(rootJoint.getSuccessor()));
      gravitationalWrench.setLinearPartZ(-gravityZ * totalMass);
      totalWrenchAlreadyApplied = new SpatialForceVector(centerOfMassFrame);

      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBody rigidBody = this.contactablePlaneBodies.get(i).getRigidBody();
         externalWrenches.put(rigidBody, new Wrench(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame()));
      }
   }

   public void reset()
   {
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBody rigidBody = contactablePlaneBodies.get(i).getRigidBody();
         externalWrenches.get(rigidBody).setToZero(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
      }

      for (int i = 0; i < rigidBodiesWithWrenchToCompensateFor.size(); i++)
      {
         RigidBody rigidBody = rigidBodiesWithWrenchToCompensateFor.get(i);
         externalWrenches.get(rigidBody).setToZero(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
      }
   }

   /**
    * Computes \dot{A} * v - external wrenches already applied (e.g. gravity, hand wrenches due to manipulation)
    *
    * @return
    */
   public final DenseMatrix64F computeWrenchEquationRightHandSide(DenseMatrix64F momentumConvectiveTerm, DenseMatrix64F b, DenseMatrix64F bHat)
   {
      totalWrenchAlreadyApplied.set(gravitationalWrench);
      for (int i = 0; i < externalWrenchesToCompensateForList.size(); i++)
      {
         tempWrench.set(externalWrenchesToCompensateForList.get(i));
         tempWrench.changeFrame(gravitationalWrench.getExpressedInFrame());
         totalWrenchAlreadyApplied.add(tempWrench);
      }

      totalWrenchAlreadyApplied.getMatrix(wrenchEquationRightHandSide);
      CommonOps.changeSign(wrenchEquationRightHandSide);
      CommonOps.addEquals(wrenchEquationRightHandSide, momentumConvectiveTerm);
      CommonOps.addEquals(wrenchEquationRightHandSide, b);
      CommonOps.subtractEquals(wrenchEquationRightHandSide, bHat);
      return wrenchEquationRightHandSide;
   }

   public DenseMatrix64F getSumOfExternalWrenches()
   {
      totalWrenchAlreadyApplied.setToZero(centerOfMassFrame);

      for (int i = 0; i < externalWrenchesToCompensateForList.size(); i++)
      {
         tempWrench.set(externalWrenchesToCompensateForList.get(i));
         tempWrench.changeFrame(centerOfMassFrame);
         totalWrenchAlreadyApplied.add(tempWrench);
      }

      totalWrenchAlreadyApplied.getMatrix(totalWrenchAlreadyAppliedMatrix);
      return totalWrenchAlreadyAppliedMatrix;
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      boolean containsRigidBody = externalWrenchesToCompensateFor.get(rigidBody) != null;
      if (!containsRigidBody)
      {
         externalWrenches.put(rigidBody, new Wrench(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame()));
         externalWrenchesToCompensateFor.put(rigidBody, new Wrench(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame()));
         externalWrenchesToCompensateForList.add(externalWrenchesToCompensateFor.get(rigidBody));
         rigidBodiesWithWrenchToCompensateFor.add(rigidBody);
      }
      ReferenceFrame bodyFixedFrame = rigidBody.getBodyFixedFrame();
      wrench.getBodyFrame().checkReferenceFrameMatch(bodyFixedFrame);
      wrench.getExpressedInFrame().checkReferenceFrameMatch(bodyFixedFrame);

      externalWrenchesToCompensateFor.get(rigidBody).set(wrench);
   }

   public void computeExternalWrenches(Map<RigidBody, Wrench> groundReactionWrenches)
   {
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBody rigidBody = contactablePlaneBodies.get(i).getRigidBody();
         Wrench externalWrench = externalWrenches.get(rigidBody);
         if(groundReactionWrenches.containsKey(rigidBody))
            externalWrench.add(groundReactionWrenches.get(rigidBody));
         if (!rigidBodiesWithExternalWrench.contains(rigidBody))
            rigidBodiesWithExternalWrench.add(rigidBody);
      }

      for (int i = 0; i < rigidBodiesWithWrenchToCompensateFor.size(); i++)
      {
         RigidBody rigidBody = rigidBodiesWithWrenchToCompensateFor.get(i);
         Wrench externalWrench = externalWrenches.get(rigidBody);
         externalWrench.add(externalWrenchesToCompensateFor.get(rigidBody));
         if (!rigidBodiesWithExternalWrench.contains(rigidBody))
            rigidBodiesWithExternalWrench.add(rigidBody);
      }
   }

   public Map<RigidBody, Wrench> getExternalWrenchMap()
   {
      return externalWrenches;
   }

   public List<RigidBody> getRigidBodiesWithExternalWrench()
   {
      return rigidBodiesWithExternalWrench;
   }

   public DenseMatrix64F getGravitationalWrench()
   {
      gravitationalWrench.getMatrix(gravitationalWrenchMatrix);
      return gravitationalWrenchMatrix;
   }
}

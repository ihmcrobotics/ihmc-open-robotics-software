package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;

/**
 * @author twan Date: 5/2/13
 */
public class ExternalWrenchHandler
{
   private final double gravityZ;
   private final RigidBodyBasics rootBody;
   private final SpatialForce gravitationalWrench;
   private final DMatrixRMaj gravitationalWrenchMatrix = new DMatrixRMaj(Wrench.SIZE, 1);
   private final DMatrixRMaj wrenchEquationRightHandSide = new DMatrixRMaj(Wrench.SIZE, 1);
   private final Map<RigidBodyBasics, Wrench> externalWrenchesToCompensateFor = new LinkedHashMap<RigidBodyBasics, Wrench>();
   /** For garbage free iteration */
   private final List<Wrench> externalWrenchesToCompensateForList = new ArrayList<Wrench>();
   private final SpatialForce totalWrenchAlreadyApplied; // gravity plus external wrenches to compensate for
   private final DMatrixRMaj totalWrenchAlreadyAppliedMatrix = new DMatrixRMaj(Wrench.SIZE, 1);
   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final List<RigidBodyBasics> rigidBodiesWithWrenchToCompensateFor = new ArrayList<>();
   private final List<RigidBodyBasics> rigidBodiesWithExternalWrench = new ArrayList<>();
   private final Map<RigidBodyBasics, Wrench> externalWrenches = new LinkedHashMap<RigidBodyBasics, Wrench>();
   private final SpatialForce tempWrench = new SpatialForce();
   private final ReferenceFrame centerOfMassFrame;

   public ExternalWrenchHandler(RigidBodyBasics rootBody, double gravityZ, ReferenceFrame centerOfMassFrame,
                                List<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      MathTools.checkIntervalContains(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.contactablePlaneBodies = new ArrayList<>(contactablePlaneBodies);

      this.gravityZ = gravityZ;
      this.rootBody = rootBody;
      gravitationalWrench = new SpatialForce(centerOfMassFrame);
      gravitationalWrench.setLinearPartZ(-gravityZ * TotalMassCalculator.computeSubTreeMass(rootBody));
      totalWrenchAlreadyApplied = new SpatialForce(centerOfMassFrame);

      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = this.contactablePlaneBodies.get(i).getRigidBody();
         externalWrenches.put(rigidBody, new Wrench(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame()));
      }
   }

   public void reset()
   {
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactablePlaneBodies.get(i).getRigidBody();
         externalWrenches.get(rigidBody).setToZero(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
      }

      for (int i = 0; i < rigidBodiesWithWrenchToCompensateFor.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodiesWithWrenchToCompensateFor.get(i);
         externalWrenches.get(rigidBody).setToZero(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
      }
   }

   /**
    * Computes \dot{A} * v - external wrenches already applied (e.g. gravity, hand wrenches due to
    * manipulation)
    *
    * @return
    */
   public final DMatrixRMaj computeWrenchEquationRightHandSide(DMatrixRMaj momentumConvectiveTerm, DMatrixRMaj b, DMatrixRMaj bHat)
   {
      totalWrenchAlreadyApplied.setIncludingFrame(gravitationalWrench);
      for (int i = 0; i < externalWrenchesToCompensateForList.size(); i++)
      {
         tempWrench.setIncludingFrame(externalWrenchesToCompensateForList.get(i));
         tempWrench.changeFrame(gravitationalWrench.getReferenceFrame());
         totalWrenchAlreadyApplied.add(tempWrench);
      }

      totalWrenchAlreadyApplied.get(wrenchEquationRightHandSide);
      CommonOps_DDRM.changeSign(wrenchEquationRightHandSide);
      CommonOps_DDRM.addEquals(wrenchEquationRightHandSide, momentumConvectiveTerm);
      CommonOps_DDRM.addEquals(wrenchEquationRightHandSide, b);
      CommonOps_DDRM.subtractEquals(wrenchEquationRightHandSide, bHat);
      return wrenchEquationRightHandSide;
   }

   public DMatrixRMaj getSumOfExternalWrenches()
   {
      totalWrenchAlreadyApplied.setToZero(centerOfMassFrame);

      for (int i = 0; i < externalWrenchesToCompensateForList.size(); i++)
      {
         tempWrench.setIncludingFrame(externalWrenchesToCompensateForList.get(i));
         tempWrench.changeFrame(centerOfMassFrame);
         totalWrenchAlreadyApplied.add(tempWrench);
      }

      totalWrenchAlreadyApplied.get(totalWrenchAlreadyAppliedMatrix);
      return totalWrenchAlreadyAppliedMatrix;
   }

   public void setExternalWrenchToCompensateFor(RigidBodyBasics rigidBody, WrenchReadOnly wrench)
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
      wrench.getReferenceFrame().checkReferenceFrameMatch(bodyFixedFrame);

      externalWrenchesToCompensateFor.get(rigidBody).setIncludingFrame(wrench);
   }

   public void computeExternalWrenches(Map<RigidBodyBasics, Wrench> groundReactionWrenches)
   {
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactablePlaneBodies.get(i).getRigidBody();
         Wrench externalWrench = externalWrenches.get(rigidBody);
         if (groundReactionWrenches.containsKey(rigidBody))
            externalWrench.add(groundReactionWrenches.get(rigidBody));
         if (!rigidBodiesWithExternalWrench.contains(rigidBody))
            rigidBodiesWithExternalWrench.add(rigidBody);
      }

      for (int i = 0; i < rigidBodiesWithWrenchToCompensateFor.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodiesWithWrenchToCompensateFor.get(i);
         Wrench externalWrench = externalWrenches.get(rigidBody);
         externalWrench.add(externalWrenchesToCompensateFor.get(rigidBody));
         if (!rigidBodiesWithExternalWrench.contains(rigidBody))
            rigidBodiesWithExternalWrench.add(rigidBody);
      }
   }

   public Map<RigidBodyBasics, Wrench> getExternalWrenchMap()
   {
      return externalWrenches;
   }

   public List<RigidBodyBasics> getRigidBodiesWithExternalWrench()
   {
      return rigidBodiesWithExternalWrench;
   }

   public DMatrixRMaj getGravitationalWrench()
   {
      gravitationalWrench.setLinearPartZ(-gravityZ * TotalMassCalculator.computeSubTreeMass(rootBody));
      gravitationalWrench.get(gravitationalWrenchMatrix);
      return gravitationalWrenchMatrix;
   }
}

package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointTorqueBasedFootSwitch implements FootSwitchInterface
{
   private final YoRegistry registry;

   private final JointTorqueBasedTouchdownDetector touchdownDetector;
   private final JacobianBasedWrenchCalculator wrenchCalculator;

   private final ReferenceFrame soleFrame;

   public JointTorqueBasedFootSwitch(String jointNameToCheck,
                                     RigidBodyBasics foot,
                                     RigidBodyBasics rootBody,
                                     ReferenceFrame soleFrame,
                                     DoubleProvider torqueContactThreshold,
                                     DoubleProvider torqueSecondContactThreshold,
                                     YoRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;
      if (rootBody == null)
      {
         throw new RuntimeException("This class needs an implementation of the root body to function!");
      }

      // we have to find the joint that we want to be checking in the kinematic chain for contact. We can't, however, pass in the full robot model without
      // significantly modifying the factory interface. So we can get it from the kinematic chain.
      OneDoFJointReadOnly jointToRead = null;
      for (OneDoFJointReadOnly candidateJoint : MultiBodySystemTools.createOneDoFJointPath(foot, rootBody))
      {
         if (candidateJoint.getName().contains(jointNameToCheck))
         {
            jointToRead = candidateJoint;
            break;
         }
      }

      if (jointToRead == null)
      {
         throw new RuntimeException("Unable to find joint " + jointNameToCheck + " in kinematic chain from " + rootBody.getName() + " to " + foot.getName());
      }

      registry = new YoRegistry(jointToRead.getName() + getClass().getSimpleName());
      touchdownDetector = new JointTorqueBasedTouchdownDetector("", jointToRead, true, torqueContactThreshold, torqueSecondContactThreshold, registry);

      wrenchCalculator = new JacobianBasedWrenchCalculator(foot, rootBody, soleFrame);

      parentRegistry.addChild(registry);
   }

   @Override
   public void reset()
   {
      touchdownDetector.reset();
   }

   @Override
   public void update()
   {
      touchdownDetector.update();

      wrenchCalculator.calculate();
   }

   @Override
   public boolean hasFootHitGroundSensitive()
   {
      return touchdownDetector.hasTouchdedDownSensitive();
   }

   /**
    * The joint has a really high torque on it.
    */
   @Override
   public boolean hasFootHitGroundFiltered()
   {
      return touchdownDetector.hasTouchedDownFiltered();
   }

   @Override
   public double getFootLoadPercentage()
   {
      return Double.NaN;
   }

   @Override
   public FramePoint2DReadOnly getCenterOfPressure()
   {
      return null;
   }

   @Override
   public WrenchReadOnly getMeasuredWrench()
   {
      return wrenchCalculator.getWrench();
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return soleFrame;
   }

   private static class JointTorqueBasedTouchdownDetector
   {
      private final OneDoFJointReadOnly joint;
      private final YoDouble jointTorque;
      private final DoubleProvider torqueThreshold;
      private final DoubleProvider torqueHigherThreshold;
      private final YoBoolean touchdownDetected;
      private final GlitchFilteredYoBoolean touchdownDetectedFiltered;
      private final YoBoolean touchdownDetectedSecondThreshold;

      private final boolean dontDetectTouchdownIfAtJointLimit;

      /**
       * @param joint joint used to detect touchdown
       * @param dontDetectTouchdownIfAtJointLimit if true, this detector will not detect a touchdown if the joint is past a joint limit. this is to avoid
       *                                          false-positive touchdown signals given by simulated torques at joint limits
       * @param registry
       */
      public JointTorqueBasedTouchdownDetector(String suffix, OneDoFJointReadOnly joint, boolean dontDetectTouchdownIfAtJointLimit,
                                               DoubleProvider torqueThreshold,
                                               DoubleProvider torqueHigherThreshold,
                                               YoRegistry registry)
      {
         this.joint = joint;
         this.dontDetectTouchdownIfAtJointLimit = dontDetectTouchdownIfAtJointLimit;
         this.torqueThreshold = torqueThreshold;
         this.torqueHigherThreshold = torqueHigherThreshold;

         jointTorque = new YoDouble(joint.getName() + "_torqueUsedForTouchdownDetection" + suffix, registry);
         touchdownDetected = new YoBoolean(joint.getName() + "_torqueBasedTouchdownDetectedF" + suffix, registry);
         touchdownDetectedFiltered = new GlitchFilteredYoBoolean(joint.getName() + "_torqueBasedTouchdownDetectedFiltered" + suffix, registry, touchdownDetected, 5);
         touchdownDetectedSecondThreshold = new YoBoolean(joint.getName() + "_torqueBasedTouchdownSecondThreshold" + suffix, registry);
      }

      public boolean hasTouchedDownFiltered()
      {
         return touchdownDetectedFiltered.getBooleanValue();
      }

      public boolean hasTouchdedDownSensitive()
      {
         return touchdownDetectedSecondThreshold.getBooleanValue();
      }

      private boolean isAtJointLimit()
      {
         double q = joint.getQ();
         double jointLimitLower = joint.getJointLimitLower();
         double jointLimitUpper = joint.getJointLimitUpper();
         return !MathTools.intervalContains(q, jointLimitLower, jointLimitUpper, false, false);
      }

      public void update()
      {
       jointTorque.set(joint.getTau());

         if (dontDetectTouchdownIfAtJointLimit && isAtJointLimit())
         {
            touchdownDetected.set(false);
            touchdownDetectedSecondThreshold.set(false);
         }
         else
         {
            touchdownDetected.set(Math.abs(joint.getTau()) > torqueThreshold.getValue());
            touchdownDetectedSecondThreshold.set(Math.abs(joint.getTau()) > torqueHigherThreshold.getValue());
         }

         touchdownDetectedFiltered.update();
      }

      public void reset()
      {
         jointTorque.set(0.0);
         touchdownDetected.set(false);
         touchdownDetectedFiltered.set(false);
         touchdownDetectedSecondThreshold.set(false);
      }

      public String getName()
      {
         return getClass().getSimpleName();
      }
   }

   private static class JacobianBasedWrenchCalculator
   {
      private final GeometricJacobian footJacobian;
      private final OneDoFJointReadOnly[] legJoints;
      private final boolean isInvertible;

      private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
      private final DMatrixRMaj torqueVector = new DMatrixRMaj(6, 1);
      private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(6);

      private final Wrench wrench = new Wrench();

      public JacobianBasedWrenchCalculator(RigidBodyBasics foot, RigidBodyBasics pelvis, ReferenceFrame soleFrame)
      {
         footJacobian = new GeometricJacobian(pelvis, foot, soleFrame);
         wrench.setToZero(ReferenceFrame.getWorldFrame());

         legJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, foot);
         isInvertible = legJoints.length == 6;

         if (!isInvertible)
            throw new RuntimeException(
                  "We can't yet use the Jacobian Based Wrench calculator, becuase the Jacobian isn't square. We need to implement this with a pseudo inverse.");
      }

      public void calculate()
      {
         footJacobian.compute();
         DMatrixRMaj jacobianMatrix = footJacobian.getJacobianMatrix();

         for (int i = 0; i < legJoints.length; i++)
            torqueVector.set(i, 0, legJoints[i].getTau());

         solver.setA(jacobianMatrix);
         CommonOps_DDRM.scale(-1.0, torqueVector);

         solver.solve(wrenchVector, torqueVector);

         wrench.setToZero(footJacobian.getJacobianFrame());
         wrench.set(wrenchVector);
      }

      public WrenchReadOnly getWrench()
      {
         return wrench;
      }
   }
}

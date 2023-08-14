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
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class JointTorqueBasedFootSwitch implements FootSwitchInterface
{
   private final YoRegistry registry;

   private final JointTorqueBasedTouchdownDetector touchdownDetector;
   private final JacobianBasedWrenchCalculator wrenchCalculator;

   private final ReferenceFrame soleFrame;

   public JointTorqueBasedFootSwitch(String namePrefix,
                                     String jointNameToCheck,
                                     RigidBodyBasics foot,
                                     RigidBodyBasics rootBody,
                                     ReferenceFrame soleFrame,
                                     DoubleProvider torqueContactThreshold,
                                     DoubleProvider torqueSecondContactThreshold,
                                     YoInteger contactThresholdWindowSize,
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
      touchdownDetector = new JointTorqueBasedTouchdownDetector(namePrefix,
                                                                jointToRead,
                                                                true,
                                                                torqueContactThreshold,
                                                                torqueSecondContactThreshold,
                                                                contactThresholdWindowSize,
                                                                registry);

      wrenchCalculator = new JacobianBasedWrenchCalculator(foot, rootBody, soleFrame, registry);

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
      return touchdownDetector.hasTouchedDownSensitive();
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
       * @param joint                             joint used to detect touchdown
       * @param dontDetectTouchdownIfAtJointLimit if true, this detector will not detect a touchdown if
       *                                          the joint is past a joint limit. this is to avoid
       *                                          false-positive touchdown signals given by simulated
       *                                          torques at joint limits
       * @param registry
       */
      public JointTorqueBasedTouchdownDetector(String namePrefix,
                                               OneDoFJointReadOnly joint,
                                               boolean dontDetectTouchdownIfAtJointLimit,
                                               DoubleProvider torqueThreshold,
                                               DoubleProvider torqueHigherThreshold,
                                               YoInteger contactThresholdWindowSize,
                                               YoRegistry registry)
      {
         this.joint = joint;
         this.dontDetectTouchdownIfAtJointLimit = dontDetectTouchdownIfAtJointLimit;
         this.torqueThreshold = torqueThreshold;
         this.torqueHigherThreshold = torqueHigherThreshold;

         jointTorque = new YoDouble(namePrefix + joint.getName() + "_torqueUsedForTouchdownDetection", registry);
         touchdownDetected = new YoBoolean(namePrefix + joint.getName() + "_torqueBasedTouchdownDetected", registry);
         touchdownDetectedFiltered = new GlitchFilteredYoBoolean(namePrefix + joint.getName() + "_torqueBasedTouchdownDetectedFiltered",
                                                                 registry,
                                                                 touchdownDetected,
                                                                 contactThresholdWindowSize);
         touchdownDetectedSecondThreshold = new YoBoolean(namePrefix + joint.getName() + "_torqueBasedTouchdownSecondThreshold", registry);
      }

      public boolean hasTouchedDownFiltered()
      {
         return touchdownDetectedFiltered.getBooleanValue();
      }

      public boolean hasTouchedDownSensitive()
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
            // This isn't an absolute value. When the robot is in support, the torque is negative, it's usually only positive in swing.
            touchdownDetected.set(joint.getTau() < -torqueThreshold.getValue());
            touchdownDetectedSecondThreshold.set(joint.getTau() < -torqueHigherThreshold.getValue());
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
      private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(6, 1);
      private final OneDoFJointReadOnly[] legJoints;
      private final boolean isInvertible;

      private final YoFixedFrameWrench yoWrench;
      private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
      private final DMatrixRMaj torqueVector = new DMatrixRMaj(6, 1);
      private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(6);

      public JacobianBasedWrenchCalculator(RigidBodyBasics foot, RigidBodyBasics pelvis, ReferenceFrame soleFrame, YoRegistry registry)
      {
         footJacobian = new GeometricJacobian(pelvis, foot, soleFrame);

         legJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, foot);
         isInvertible = legJoints.length == 6;

         if (!isInvertible)
            throw new RuntimeException("We can't yet use the Jacobian Based Wrench calculator, becuase the Jacobian isn't square. We need to implement this with a pseudo inverse.");

         yoWrench = new YoFixedFrameWrench(foot.getBodyFixedFrame(),
                                           new YoFrameVector3D(foot.getName() + "EstimatedFootTorque", soleFrame, registry),
                                           new YoFrameVector3D(foot.getName() + "EstimatedFootForce", soleFrame, registry));
      }

      public void calculate()
      {
         footJacobian.compute();
         jacobianTranspose.reshape(footJacobian.getNumberOfColumns(), 6);
         CommonOps_DDRM.transpose(footJacobian.getJacobianMatrix(), jacobianTranspose);

         for (int i = 0; i < legJoints.length; i++)
            torqueVector.set(i, 0, legJoints[i].getTau());

         solver.setA(jacobianTranspose);
         CommonOps_DDRM.scale(-1.0, torqueVector);

         solver.solve(torqueVector, wrenchVector);
         yoWrench.set(wrenchVector);
      }

      public WrenchReadOnly getWrench()
      {
         return yoWrench;
      }
   }
}

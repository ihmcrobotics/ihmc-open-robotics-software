package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class JointTorqueBasedFootSwitch implements FootSwitchInterface
{
   private final YoRegistry registry;

   private final BooleanProvider useJacobianTranspose;
   private final JointTorqueBasedTouchdownDetector touchdownDetector;
   private final JacobianBasedBasedTouchdownDetector wrenchDetector;

   private final MovingReferenceFrame soleFrame;

   public JointTorqueBasedFootSwitch(String namePrefix,
                                     String jointNameToCheck,
                                     RigidBodyBasics foot,
                                     RigidBodyBasics rootBody,
                                     MovingReferenceFrame soleFrame,
                                     DoubleProvider torqueContactThreshold,
                                     DoubleProvider torqueSecondContactThreshold,
                                     DoubleProvider contactForceThreshold,
                                     YoInteger contactThresholdWindowSize,
                                     BooleanProvider compensateGravity,
                                     DoubleProvider horizontalVelocityThreshold,
                                     DoubleProvider verticalVelocityThreshold,
                                     BooleanProvider useJacobianTranspose,
                                     YoRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;
      this.useJacobianTranspose = useJacobianTranspose;

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

      wrenchDetector = new JacobianBasedBasedTouchdownDetector(foot,
                                                               rootBody,
                                                               soleFrame,
                                                               TotalMassCalculator.computeSubTreeMass(MultiBodySystemTools.getRootBody(rootBody)),
                                                               contactForceThreshold,
                                                               contactThresholdWindowSize,
                                                               compensateGravity,
                                                               horizontalVelocityThreshold,
                                                               verticalVelocityThreshold,
                                                               registry);

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

      wrenchDetector.calculate();
   }

   @Override
   public boolean hasFootHitGroundSensitive()
   {
      if (useJacobianTranspose.getValue())
         return wrenchDetector.hasFootHitGroundSensitive();
      else
         return touchdownDetector.hasTouchedDownSensitive();
   }

   /**
    * The joint has a really high torque on it.
    */
   @Override
   public boolean hasFootHitGroundFiltered()
   {
      if (useJacobianTranspose.getValue())
         return wrenchDetector.hasFootHitGroundFiltered();
      else
         return touchdownDetector.hasTouchedDownFiltered();
   }

   @Override
   public double getFootLoadPercentage()
   {
      if (useJacobianTranspose.getValue())
         return wrenchDetector.getFootLoadPercentage();
      else
         return Double.NaN;
   }

   @Override
   public FramePoint2DReadOnly getCenterOfPressure()
   {
      if (useJacobianTranspose.getValue())
         return wrenchDetector.getCenterOfPressure();
      else
         return null;
   }

   @Override
   public WrenchReadOnly getMeasuredWrench()
   {
      if (useJacobianTranspose.getValue())
         return wrenchDetector.getWrench();
      else
         return null;
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
   }

   private static class JacobianBasedBasedTouchdownDetector
   {
      private static final double MIN_FORCE_TO_COMPUTE_COP = 5.0;

      private final MovingReferenceFrame soleFrame;
      private final GeometricJacobian footJacobian;
      private final InverseDynamicsCalculator gravityTorqueCalculator;
      private final DMatrixRMaj jacobianTranspose = new DMatrixRMaj(6, 1);
      private final OneDoFJointReadOnly[] legJoints;
      private final YoDouble[] legJointGravityTaus;

      private final YoFixedFrameWrench wrench;
      private final YoFixedFrameWrench wrenchNoGravity;
      private final DMatrixRMaj wrenchVector = new DMatrixRMaj(6, 1);
      private final DMatrixRMaj torqueVector = new DMatrixRMaj(6, 1);
      private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(6);

      private final YoFrameVector3D linearVelocity;
      private final YoDouble horizontalVelocity, verticalVelocity;

      private final BooleanProvider compensateGravity;

      private final double robotTotalWeight;
      private final DoubleProvider contactForceThreshold;
      private final DoubleProvider horizontalVelocityThreshold;
      private final DoubleProvider verticalVelocityThreshold;
      private final YoBoolean isPastForceThreshold;
      private final YoBoolean hasFootHitGround;
      private final GlitchFilteredYoBoolean hasFootHitGroundFiltered;

      private final YoDouble footForceMagnitude;
      private final YoDouble alphaFootLoadFiltering;
      private final AlphaFilteredYoVariable footLoadPercentage;

      private final YoFramePoint2D centerOfPressure;
      private final CenterOfPressureResolver copResolver = new CenterOfPressureResolver();

      public JacobianBasedBasedTouchdownDetector(RigidBodyBasics foot,
                                                 RigidBodyBasics pelvis,
                                                 MovingReferenceFrame soleFrame,
                                                 double robotTotalWeight,
                                                 DoubleProvider contactForceThreshold,
                                                 YoInteger contactThresholdWindowSize,
                                                 BooleanProvider compensateGravity,
                                                 DoubleProvider horizontalVelocityThreshold,
                                                 DoubleProvider verticalVelocityThreshold,
                                                 YoRegistry registry)
      {
         this.soleFrame = soleFrame;
         this.robotTotalWeight = robotTotalWeight;
         this.contactForceThreshold = contactForceThreshold;
         this.compensateGravity = compensateGravity;
         this.horizontalVelocityThreshold = horizontalVelocityThreshold;
         this.verticalVelocityThreshold = verticalVelocityThreshold;

         legJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, foot);

         if (legJoints.length != 6)
            throw new RuntimeException("We can't yet use the Jacobian Based Wrench calculator, because the Jacobian isn't square. We need to implement this with a pseudo inverse.");

         footJacobian = new GeometricJacobian(pelvis, foot, soleFrame);
         gravityTorqueCalculator = new InverseDynamicsCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(legJoints));
         gravityTorqueCalculator.setConsiderJointAccelerations(false);
         gravityTorqueCalculator.setGravitionalAcceleration(-9.81); // TODO Extract me

         legJointGravityTaus = new YoDouble[6];
         for (int i = 0; i < legJointGravityTaus.length; i++)
         {
            legJointGravityTaus[i] = new YoDouble("tau_gravity_" + legJoints[i].getName(), registry);
         }
         String namePrefix = foot.getName() + "JTrans";

         wrench = new YoFixedFrameWrench(foot.getBodyFixedFrame(),
                                         new YoFrameVector3D(namePrefix + "EstimatedTorque", soleFrame, registry),
                                         new YoFrameVector3D(namePrefix + "EstimatedForce", soleFrame, registry));
         wrenchNoGravity = new YoFixedFrameWrench(foot.getBodyFixedFrame(),
                                                  new YoFrameVector3D(namePrefix + "EstimatedTorqueNoGravity", soleFrame, registry),
                                                  new YoFrameVector3D(namePrefix + "EstimatedForceNoGravity", soleFrame, registry));

         linearVelocity = new YoFrameVector3D(namePrefix + "LinearVelocity", ReferenceFrame.getWorldFrame(), registry);
         horizontalVelocity = new YoDouble(namePrefix + "HorizontalVelocity", registry);
         verticalVelocity = new YoDouble(namePrefix + "VerticalVelocity", registry);

         isPastForceThreshold = new YoBoolean(namePrefix + "IsPastForceThreshold", registry);
         footForceMagnitude = new YoDouble(namePrefix + "FootForceMag", registry);

         alphaFootLoadFiltering = new YoDouble(namePrefix + "AlphaFootLoadFiltering", registry);
         alphaFootLoadFiltering.set(0.1);
         footLoadPercentage = new AlphaFilteredYoVariable(namePrefix + "FootLoadPercentage", registry, alphaFootLoadFiltering);

         hasFootHitGround = new YoBoolean(namePrefix + "FootHitGround", registry);
         // Final variable to identify if the foot has hit the ground
         hasFootHitGroundFiltered = new GlitchFilteredYoBoolean(namePrefix + "HasFootHitGroundFiltered",
                                                                registry,
                                                                hasFootHitGround,
                                                                contactThresholdWindowSize);

         centerOfPressure = new YoFramePoint2D(namePrefix + "CenterOfPressure", "", soleFrame, registry);
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
         wrench.set(wrenchVector);

         gravityTorqueCalculator.compute();

         for (int i = 0; i < legJoints.length; i++)
         {
            legJointGravityTaus[i].set(gravityTorqueCalculator.getComputedJointTau(legJoints[i]).get(0));
            torqueVector.set(i, 0, legJoints[i].getTau() - legJointGravityTaus[i].getValue());
         }

         CommonOps_DDRM.scale(-1.0, torqueVector);

         solver.solve(torqueVector, wrenchVector);
         wrenchNoGravity.set(wrenchVector);

         updateFootSwitch(compensateGravity.getValue() ? wrenchNoGravity : wrench);
      }

      private void updateFootSwitch(WrenchReadOnly wrench)
      {
         footForceMagnitude.set(wrench.getLinearPart().norm());

         // Using the force in foot frame to ensure z is up when the foot is flat.
         // Sometimes the sensor can be mounted such that z is down.
         double forceZUp = wrench.getLinearPartZ();

         double fZPlus = MathTools.clamp(forceZUp, 0.0, Double.POSITIVE_INFINITY);
         footLoadPercentage.update(fZPlus / robotTotalWeight);

         isPastForceThreshold.set(forceZUp > contactForceThreshold.getValue());

         // Looking at velocity thresholds
         linearVelocity.setMatchingFrame(soleFrame.getTwistOfFrame().getLinearPart());
         horizontalVelocity.set(EuclidCoreTools.norm(linearVelocity.getX(), linearVelocity.getY()));
         verticalVelocity.set(linearVelocity.getZ());

         hasFootHitGround.set(isPastForceThreshold.getValue() && horizontalVelocity.getValue() < horizontalVelocityThreshold.getValue()
                              && Math.abs(verticalVelocity.getValue()) < verticalVelocityThreshold.getValue());
         hasFootHitGroundFiltered.update();

         // Computing Center of Pressure
         if (fZPlus < MIN_FORCE_TO_COMPUTE_COP)
            centerOfPressure.setToNaN();
         else
            copResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure, wrench, soleFrame);
      }

      public boolean hasFootHitGroundSensitive()
      {
         return hasFootHitGround.getValue();
      }

      public boolean hasFootHitGroundFiltered()
      {
         return hasFootHitGroundFiltered.getValue();
      }

      public WrenchReadOnly getWrench()
      {
         return compensateGravity.getValue() ? wrench : wrenchNoGravity;
      }

      public double getFootLoadPercentage()
      {
         return footLoadPercentage.getDoubleValue();
      }

      public FramePoint2DReadOnly getCenterOfPressure()
      {
         return centerOfPressure;
      }
   }
}

package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
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
import us.ihmc.robotics.MultiBodySystemMissingTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

public class JointTorqueBasedFootSwitch implements FootSwitchInterface
{
   private static final double GRAVITY_Z = -9.81;
   private static final int FORCE_THRESHOLD_WINDOW_SIZE = 2;
   private static final int COP_THRESHOLD_WINDOW_SIZE = 3;
   private static final int MINIMUM_WINDOW_SIZE = Math.min(FORCE_THRESHOLD_WINDOW_SIZE, COP_THRESHOLD_WINDOW_SIZE);
   private final YoRegistry registry;

   private final BooleanProvider useJacobianTranspose;
   private final JointTorqueBasedTouchdownDetector touchdownDetector;
   private final JacobianBasedBasedTouchdownDetector wrenchDetector;

   private final YoInteger contactThresholdWindowSize;
   private final YoDouble contactThresholdWindowDuration;
   private final DoubleProvider switchDT;

   private final MovingReferenceFrame soleFrame;

   public JointTorqueBasedFootSwitch(String namePrefix,
                                     String jointNameToCheck,
                                     RigidBodyBasics rootBody,
                                     ContactablePlaneBody contactablePlaneBody,
                                     DoubleProvider torqueContactThreshold,
                                     DoubleProvider torqueSecondContactThreshold,
                                     DoubleProvider contactForceThresholdLow,
                                     DoubleProvider contactForceThresholdHigh,
                                     DoubleProvider contactCoPThreshold,
                                     double contactWindowDuration,
                                     DoubleProvider switchDT,
                                     BooleanProvider compensateGravity,
                                     DoubleProvider horizontalVelocityThreshold,
                                     DoubleProvider verticalVelocityThreshold,
                                     DoubleProvider verticalVelocityHighThreshold,
                                     DoubleProvider jacobianDeterminantSingularityThreshold,
                                     BooleanProvider useJacobianTranspose,
                                     YoRegistry parentRegistry)
   {
      this.useJacobianTranspose = useJacobianTranspose;

      soleFrame = (MovingReferenceFrame) contactablePlaneBody.getContactFrame();

      RigidBodyBasics foot = contactablePlaneBody.getRigidBody();

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

      this.switchDT = switchDT;

      contactThresholdWindowDuration = new YoDouble(namePrefix + "ContactThresholdWindowDuration", registry);
      contactThresholdWindowDuration.set(contactWindowDuration);
      contactThresholdWindowSize = new YoInteger(namePrefix + "ContactThresholdWindowSize", registry);
      contactThresholdWindowSize.set(Math.max(MINIMUM_WINDOW_SIZE, (int) Math.ceil(contactWindowDuration / switchDT.getValue())));

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
                                                               MultiBodySystemMissingTools.computeSubTreeMass(MultiBodySystemTools.getRootBody(rootBody)) * Math.abs(GRAVITY_Z),
                                                               contactablePlaneBody,
                                                               contactForceThresholdLow,
                                                               contactForceThresholdHigh,
                                                               contactCoPThreshold,
                                                               contactThresholdWindowSize,
                                                               compensateGravity,
                                                               horizontalVelocityThreshold,
                                                               verticalVelocityThreshold,
                                                               verticalVelocityHighThreshold,
                                                               jacobianDeterminantSingularityThreshold,
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
      // Account for any changes in the DT by updating the window size to maintain a consistent time window for contact detection.
      contactThresholdWindowSize.set(Math.max(MINIMUM_WINDOW_SIZE, (int) Math.ceil(contactThresholdWindowDuration.getDoubleValue() / switchDT.getValue())));

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
   public double getCenterOfPressureDistance()
   {
      if (useJacobianTranspose.getValue())
         return wrenchDetector.getCenterOfPressureDistance();
      else
         return Double.NaN;
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
      private final DoubleProvider contactForceThresholdLow;
      private final DoubleProvider contactForceThresholdHigh;
      private final DoubleProvider contactCoPThreshold;

      private final DoubleProvider horizontalVelocityThreshold;
      private final DoubleProvider verticalVelocityThreshold;
      private final DoubleProvider verticalVelocityHighThreshold;
      private final DoubleProvider jacobianDeterminantThreshold;
      private final YoBoolean isPastForceThresholdLow;
      private final GlitchFilteredYoBoolean isPastForceThresholdLowFiltered;
      private final YoBoolean isPastForceThresholdHigh;
      private final YoBoolean hasFootHitGround, isPastCoPThreshold;
      private final GlitchFilteredYoBoolean hasFootHitGroundFiltered;
      private final GlitchFilteredYoBoolean isPastCoPThresholdFiltered;

      private final YoDouble jacobianDeterminant;
      private final YoDouble copDistance;
      private final YoDouble footForceMagnitude;
      private final YoDouble alphaFootLoadFiltering;
      private final AlphaFilteredYoVariable footLoadPercentage;

      private final YoFramePoint2D centerOfPressure;
      private final CenterOfPressureResolver copResolver = new CenterOfPressureResolver();
      private final FrameConvexPolygon2D footPolygon;

      public JacobianBasedBasedTouchdownDetector(RigidBodyBasics foot,
                                                 RigidBodyBasics pelvis,
                                                 MovingReferenceFrame soleFrame,
                                                 double robotTotalWeight,
                                                 ContactablePlaneBody contactablePlaneBody,
                                                 DoubleProvider contactForceThresholdLow,
                                                 DoubleProvider contactForceThresholdHigh,
                                                 DoubleProvider contactCoPThreshold,
                                                 YoInteger contactThresholdWindowSize,
                                                 BooleanProvider compensateGravity,
                                                 DoubleProvider horizontalVelocityThreshold,
                                                 DoubleProvider verticalVelocityThreshold,
                                                 DoubleProvider verticalVelocityHighThreshold,
                                                 DoubleProvider jacobianDeterminantThreshold,
                                                 YoRegistry registry)
      {
         this.soleFrame = soleFrame;
         this.robotTotalWeight = robotTotalWeight;
         this.contactForceThresholdLow = contactForceThresholdLow;
         this.contactForceThresholdHigh = contactForceThresholdHigh;
         this.contactCoPThreshold = contactCoPThreshold;
         this.compensateGravity = compensateGravity;
         this.horizontalVelocityThreshold = horizontalVelocityThreshold;
         this.verticalVelocityThreshold = verticalVelocityThreshold;
         this.verticalVelocityHighThreshold = verticalVelocityHighThreshold;
         this.jacobianDeterminantThreshold = jacobianDeterminantThreshold;

         legJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, foot);

         if (legJoints.length != 6)
            throw new RuntimeException("We can't yet use the Jacobian Based Wrench calculator, because the Jacobian isn't square. We need to implement this with a pseudo inverse.");

         footJacobian = new GeometricJacobian(pelvis, foot, soleFrame);
         gravityTorqueCalculator = new InverseDynamicsCalculator(MultiBodySystemReadOnly.toMultiBodySystemInput(legJoints));
         gravityTorqueCalculator.setConsiderJointAccelerations(false);
         gravityTorqueCalculator.setGravitionalAcceleration(GRAVITY_Z);

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

         isPastForceThresholdLow = new YoBoolean(namePrefix + "IsPastForceThresholdLow", registry);
         isPastForceThresholdLowFiltered = new GlitchFilteredYoBoolean(namePrefix + "IsPastForceThresholdLowFiltered", registry, isPastForceThresholdLow,
                                                                       FORCE_THRESHOLD_WINDOW_SIZE);
         isPastForceThresholdHigh = new YoBoolean(namePrefix + "IsPastForceThresholdHigh", registry);
         isPastCoPThreshold = new YoBoolean(namePrefix + "IsPastCoPThreshold", registry);
         isPastCoPThresholdFiltered = new GlitchFilteredYoBoolean(namePrefix + "IsPastCoPThresholdFiltered", registry, isPastCoPThreshold,
                                                                  COP_THRESHOLD_WINDOW_SIZE);

         footForceMagnitude = new YoDouble(namePrefix + "FootForceMag", registry);
         copDistance = new YoDouble(namePrefix + "CoPDistance", registry);
         jacobianDeterminant = new YoDouble(namePrefix + "JacobianDeterminant", registry);

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

         List<? extends FramePoint2DReadOnly> contactPoints = contactablePlaneBody.getContactPoints2D();
         footPolygon = new FrameConvexPolygon2D(soleFrame, Vertex2DSupplier.asVertex2DSupplier(contactPoints));
      }

      public void calculate()
      {
         footJacobian.compute();
         jacobianTranspose.reshape(footJacobian.getNumberOfColumns(), 6);
         CommonOps_DDRM.transpose(footJacobian.getJacobianMatrix(), jacobianTranspose);

         // Compute the determinant of the jacobian to help evaluate singular configurations.
         jacobianDeterminant.set(CommonOps_DDRM.det(jacobianTranspose));

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

         isPastForceThresholdLow.set(forceZUp > contactForceThresholdLow.getValue());
         isPastForceThresholdLowFiltered.update();

         if (contactForceThresholdHigh != null)
         {
            isPastForceThresholdHigh.set(forceZUp > contactForceThresholdHigh.getValue());
         }
         else
         {
            isPastForceThresholdHigh.set(false);
         }

         // Computing Center of Pressure
         if (fZPlus < MIN_FORCE_TO_COMPUTE_COP)
            centerOfPressure.setToNaN();
         else
            copResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure, wrench, soleFrame);

         // Testing CoP threshold
         if (Double.isNaN(contactCoPThreshold.getValue()))
         {
            isPastCoPThreshold.set(true);
            isPastCoPThresholdFiltered.set(true);
            copDistance.setToNaN();
         }
         else
         {
            double copThreshold = contactCoPThreshold.getValue();
            copDistance.set(footPolygon.signedDistance(centerOfPressure));
            isPastCoPThreshold.set(copDistance.getDoubleValue() < -copThreshold);
            isPastCoPThresholdFiltered.update();
         }

         // Looking at velocity thresholds
         linearVelocity.setMatchingFrame(soleFrame.getTwistOfFrame().getLinearPart());
         horizontalVelocity.set(EuclidCoreTools.norm(linearVelocity.getX(), linearVelocity.getY()));
         verticalVelocity.set(linearVelocity.getZ());

         if (jacobianDeterminant.getDoubleValue() > jacobianDeterminantThreshold.getValue())
         { // The jacobian determinant is above the threshold, so it's not in a singular configuration
            boolean validCoP = isPastCoPThresholdFiltered.getValue();
            boolean hitGroundLow = isPastForceThresholdLowFiltered.getValue() && validCoP;
            boolean allowableSpeed = horizontalVelocity.getValue() < horizontalVelocityThreshold.getValue()
                                     && Math.abs(verticalVelocity.getValue()) < verticalVelocityThreshold.getValue();
            boolean allowableHighSpeed = Math.abs(verticalVelocity.getValue()) < verticalVelocityHighThreshold.getValue();

            hasFootHitGround.set((hitGroundLow && allowableSpeed) || (isPastForceThresholdHigh.getValue() && allowableHighSpeed));
         }
         else
         { // The jacobian determinant is below the threshold, so the system is in a singular configuration. This means the forces can't be trusted.
            boolean allowableSpeed = horizontalVelocity.getValue() < horizontalVelocityThreshold.getValue()
                                     && Math.abs(verticalVelocity.getValue()) < verticalVelocityThreshold.getValue();
            hasFootHitGround.set(allowableSpeed);
         }
         hasFootHitGroundFiltered.update();
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

      public double getCenterOfPressureDistance()
      {
         return copDistance.getDoubleValue();
      }
   }
}

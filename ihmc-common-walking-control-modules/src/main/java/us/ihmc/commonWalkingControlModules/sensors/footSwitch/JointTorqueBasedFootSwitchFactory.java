package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointTorqueBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdTorque = 50.0;
   private double defaultHigherContactThresholdTorque = 100.0;
   private double defaultContactThresholdForceLow = 50.0;
   private double defaultContactThresholdForceHigh = 100;
   private double defaultContactCoPThreshold = 0.01;
   private double defaultContactWindowDuration = 0.05; // previous WBCC-specific window size = 0.03s, estimator-specific window size = 0.025s
   private boolean defaultUseJacobianTranspose = false;
   private double defaultHorizontalVelocityThreshold = 0.5;
   private double defaultVerticalVelocityThreshold = 0.125;
   private double defaultVerticalVelocityHighThreshold = 0.3;
   private double defaultJacobianDeterminantSingularityThreshold = 2e-3;
   // The historical behavior: velocity gate on, no inertial compensation, no relative check.
   private boolean defaultUseVelocityGate = true;
   private boolean defaultCompensateInertia = false;
   private double defaultInertiaAccelerationBreakFrequency = 25.0;
   private boolean defaultUseRelativeVelocityCheck = false;
   private double defaultRelativeHorizontalVelocityThreshold = 0.3;
   private double defaultRelativeVerticalVelocityThreshold = 0.1;
   private double defaultReleaseLoadFraction = Double.NaN; // off

   private DoubleProvider contactThresholdTorque;
   private DoubleProvider higherContactThresholdTorque;
   private DoubleProvider contactForceThresholdLow;
   private DoubleProvider contactForceThresholdHigh;
   private DoubleProvider contactCoPThreshold;
   private BooleanProvider compensateGravity;
   private DoubleProvider horizontalVelocityThreshold;
   private DoubleProvider verticalVelocityThreshold;
   private DoubleProvider verticalVelocityHighThreshold;
   private DoubleProvider jacobianDeterminantSingularityThreshold;
   private BooleanProvider useJacobianTranspose;
   private BooleanProvider useVelocityGate;
   private BooleanProvider compensateInertia;
   private DoubleProvider inertiaAccelerationBreakFrequency;
   private BooleanProvider useRelativeVelocityCheck;
   private DoubleProvider relativeHorizontalVelocityThreshold;
   private DoubleProvider relativeVerticalVelocityThreshold;
   private DoubleProvider releaseLoadFraction;

   /** Switches built so far, to pair each with the other foot's for the relative velocity check. */
   private final List<JointTorqueBasedFootSwitch> createdSwitches = new ArrayList<>();
   private final List<ContactablePlaneBody> createdFeet = new ArrayList<>();
   private final List<YoRegistry> createdRegistries = new ArrayList<>();

   private final String jointDescriptionToCheck;

   public JointTorqueBasedFootSwitchFactory(String jointDescriptionToCheck)
   {
      this.jointDescriptionToCheck = jointDescriptionToCheck;
   }

   /**
    * When determining whether a foot has hit the ground the controller can look at the knee torque.
    * This value is then glitch filtered to verify.
    */
   public void setDefaultContactThresholdTorque(double defaultContactThresholdTorque)
   {
      this.defaultContactThresholdTorque = defaultContactThresholdTorque;
   }

   /**
    * When determining whether a foot has hit the ground the controller can look at the knee torque.
    * This value is a higher threshold which instantaneously triggers the touchdown event.
    */
   public void setDefaultHigherContactThresholdTorque(double defaultHigherContactThresholdTorque)
   {
      this.defaultHigherContactThresholdTorque = defaultHigherContactThresholdTorque;
   }

   public void setDefaultContactWindowDuration(double window)
   {
      this.defaultContactWindowDuration = window;
   }

   /**
    * Call either {@link #setDefaultContactThresholdForceLow} or {@link #setDefaultContactThresholdForceHigh(double)}. This now links to
    * {@link #setDefaultContactThresholdForceLow(double)}
    * @param defaultContactThresholdForce
    */
   @Deprecated
   public void setDefaultContactThresholdForce(double defaultContactThresholdForce)
   {
      setDefaultContactThresholdForceLow(defaultContactThresholdForce);
   }

   public void setDefaultContactThresholdForceLow(double defaultContactThresholdForce)
   {
      this.defaultContactThresholdForceLow = defaultContactThresholdForce;
   }

   public void setDefaultContactThresholdForceHigh(double defaultContactThresholdForce)
   {
      this.defaultContactThresholdForceHigh = defaultContactThresholdForce;
   }

   public void setDefaultCoPThresholdDistance(double defaultContactCoPThreshold)
   {
      this.defaultContactCoPThreshold = defaultContactCoPThreshold;
   }

   public void setDefaultUseJacobianTranspose(boolean defaultUseJacobianTranspose)
   {
      this.defaultUseJacobianTranspose = defaultUseJacobianTranspose;
   }

   public void setDefaultHorizontalVelocityThreshold(double defaultHorizontalVelocityThreshold)
   {
      this.defaultHorizontalVelocityThreshold = defaultHorizontalVelocityThreshold;
   }

   public void setDefaultVerticalVelocityThreshold(double defaultVerticalVelocityThreshold)
   {
      this.defaultVerticalVelocityThreshold = defaultVerticalVelocityThreshold;
   }

   public void setDefaultJacobianDeterminantSingularityThreshold(double defaultJacobianDeterminantSingularityThreshold)
   {
      this.defaultJacobianDeterminantSingularityThreshold = defaultJacobianDeterminantSingularityThreshold;
   }

   /** Whether contact also requires a small sole velocity. That velocity comes from the owning estimator's root twist. */
   public void setDefaultUseVelocityGate(boolean defaultUseVelocityGate)
   {
      this.defaultUseVelocityGate = defaultUseVelocityGate;
   }

   /** Solve the foot wrench from {@code tau - ID(q, qd, qdd)} of the leg rather than {@code tau - g(q)}. */
   public void setDefaultCompensateInertia(boolean defaultCompensateInertia)
   {
      this.defaultCompensateInertia = defaultCompensateInertia;
   }

   /** Break frequency of the filter on the finite-difference joint accelerations used by the inertial compensation. */
   public void setDefaultInertiaAccelerationBreakFrequency(double breakFrequency)
   {
      this.defaultInertiaAccelerationBreakFrequency = breakFrequency;
   }

   /** Reject the less-loaded of two force-loaded feet whose soles move apart. Reads no estimated linear velocity. */
   public void setDefaultUseRelativeVelocityCheck(boolean defaultUseRelativeVelocityCheck)
   {
      this.defaultUseRelativeVelocityCheck = defaultUseRelativeVelocityCheck;
   }

   /** Release contact at once when the foot's vertical force drops below this fraction of body weight; NaN = off. */
   public void setDefaultReleaseLoadFraction(double fraction)
   {
      this.defaultReleaseLoadFraction = fraction;
   }

   public void setDefaultRelativeVelocityThresholds(double horizontal, double vertical)
   {
      this.defaultRelativeHorizontalVelocityThreshold = horizontal;
      this.defaultRelativeVerticalVelocityThreshold = vertical;
   }

   /**
    * Contact detection that does not read the owning estimator's linear velocity: velocity gate off,
    * inertial compensation on. For an estimator whose velocity would otherwise gate its own contacts --
    * the invariant filter as main estimator. The gate and the relative velocity check stay parameters
    * ({@code <prefix>UseVelocityGate}, {@code <prefix>UseRelativeVelocityCheck}) and can be switched on.
    * <p>
    * The relative check is off by default because it measured worse: on the 2026-09-28 Alex RL log
    * (invariant arm 4, walking time in |v| > 1 m/s episodes) gate on 18.3%, gate off 9.5%, + inertia 5.1%,
    * + inertia + relative check 16.1%.
    */
   public void useEstimatorIndependentDetection()
   {
      setDefaultUseVelocityGate(false);
      setDefaultCompensateInertia(true);
      setDefaultUseRelativeVelocityCheck(false);
   }

   @Override
   public FootSwitchInterface newFootSwitch(String namePrefix,
                                            ContactablePlaneBody foot,
                                            Collection<? extends ContactablePlaneBody> otherFeet,
                                            RigidBodyBasics rootBody,
                                            ForceSensorDataReadOnly footForceSensor,
                                            double totalRobotWeight,
                                            DoubleProvider switchDT,
                                            YoRegistry registry)
   {
      if (contactThresholdTorque == null)
      {
         contactThresholdTorque = new DoubleParameter(namePrefix + "ContactThresholdJointTorque", registry, defaultContactThresholdTorque);
         higherContactThresholdTorque = new DoubleParameter(namePrefix + "HigherContactThresholdJointTorque", registry, defaultHigherContactThresholdTorque);
         contactForceThresholdLow = new DoubleParameter(namePrefix + "JacobianTThresholdForceLow", registry, defaultContactThresholdForceLow);
         contactForceThresholdHigh = new DoubleParameter(namePrefix + "JacobianTThresholdForceHigh", registry, defaultContactThresholdForceHigh);
         contactCoPThreshold = new DoubleParameter(namePrefix + "JacobianTThresholdContactCoP", registry, defaultContactCoPThreshold);
         compensateGravity = new BooleanParameter(namePrefix + "JacobianTCompensateGravity", registry, true);
         useJacobianTranspose = new BooleanParameter(namePrefix + "UseJacobianTranspose", registry, defaultUseJacobianTranspose);
         verticalVelocityThreshold = new DoubleParameter(namePrefix + "VerticalVelocityThreshold", registry, defaultVerticalVelocityThreshold);
         verticalVelocityHighThreshold = new DoubleParameter(namePrefix + "VerticalVelocityHighThreshold", registry, defaultVerticalVelocityHighThreshold);
         horizontalVelocityThreshold = new DoubleParameter(namePrefix + "HorizontalVelocityThreshold", registry, defaultHorizontalVelocityThreshold);
         jacobianDeterminantSingularityThreshold = new DoubleParameter(namePrefix + "JacobianDeterminantSingularityThreshold", registry,
                                                                       defaultJacobianDeterminantSingularityThreshold);
         useVelocityGate = new BooleanParameter(namePrefix + "UseVelocityGate", registry, defaultUseVelocityGate);
         compensateInertia = new BooleanParameter(namePrefix + "CompensateInertia", registry, defaultCompensateInertia);
         inertiaAccelerationBreakFrequency = new DoubleParameter(namePrefix + "InertiaAccelerationBreakFrequency", registry,
                                                                 defaultInertiaAccelerationBreakFrequency);
         useRelativeVelocityCheck = new BooleanParameter(namePrefix + "UseRelativeVelocityCheck", registry, defaultUseRelativeVelocityCheck);
         relativeHorizontalVelocityThreshold = new DoubleParameter(namePrefix + "RelativeHorizontalVelocityThreshold", registry,
                                                                   defaultRelativeHorizontalVelocityThreshold);
         relativeVerticalVelocityThreshold = new DoubleParameter(namePrefix + "RelativeVerticalVelocityThreshold", registry,
                                                                 defaultRelativeVerticalVelocityThreshold);
         releaseLoadFraction = new DoubleParameter(namePrefix + "ReleaseLoadFraction", registry, defaultReleaseLoadFraction);
      }

      JointTorqueBasedFootSwitch footSwitch = new JointTorqueBasedFootSwitch(namePrefix,
                                            jointDescriptionToCheck,
                                            rootBody,
                                            foot,
                                            contactThresholdTorque,
                                            higherContactThresholdTorque,
                                            contactForceThresholdLow,
                                            contactForceThresholdHigh,
                                            contactCoPThreshold,
                                            defaultContactWindowDuration,
                                            switchDT,
                                            compensateGravity,
                                            horizontalVelocityThreshold,
                                            verticalVelocityThreshold,
                                            verticalVelocityHighThreshold,
                                            jacobianDeterminantSingularityThreshold,
                                            useJacobianTranspose,
                                            useVelocityGate,
                                            compensateInertia,
                                            inertiaAccelerationBreakFrequency,
                                            useRelativeVelocityCheck,
                                            relativeHorizontalVelocityThreshold,
                                            relativeVerticalVelocityThreshold,
                                            releaseLoadFraction,
                                            registry);

      // Pair with an earlier switch of this factory whose foot is one of this foot's other feet, in the
      // same registry: a biped's two switches of one estimator. Other estimators' feet are other objects.
      for (int i = 0; i < createdSwitches.size(); i++)
      {
         if (createdRegistries.get(i) == registry && otherFeet != null && otherFeet.contains(createdFeet.get(i)))
         {
            footSwitch.setOtherFootSwitch(createdSwitches.get(i));
            createdSwitches.get(i).setOtherFootSwitch(footSwitch);
         }
      }
      createdSwitches.add(footSwitch);
      createdFeet.add(foot);
      createdRegistries.add(registry);
      return footSwitch;
   }
}
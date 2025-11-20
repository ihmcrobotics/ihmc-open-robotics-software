package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.Collection;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class WrenchBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdForce = Double.NaN;
   private double defaultCoPThresholdDistance = Double.NaN;
   private double defaultSecondContactThresholdForceIgnoringCoP = Double.NaN;

   private DoubleProvider contactThresholdForceParameter;
   private DoubleProvider copThresholdFractionParameter;
   private DoubleProvider secondContactThresholdForceParameter;

   public WrenchBasedFootSwitchFactory()
   {
   }

   /**
    * When determining that a foot has hit the floor after a step the z-force on the foot needs to be
    * past the threshold defined by this method. In addition the center of pressure needs to be inside
    * certain bounds of the foot (see {@link #setDefaultCoPThresholdDistance(double)}).
    * </p>
    * See also {@link #setDefaultSecondContactThresholdForceIgnoringCoP(double)} for another threshold
    * on the contact force that does not require the CoP to be within bounds.
    * </p>
    */
   public void setDefaultContactThresholdForce(double defaultContactThresholdForce)
   {
      this.defaultContactThresholdForce = defaultContactThresholdForce;
   }

   /**
    * When determining whether a foot has touched down after a step the controller will make sure that
    * the CoP of the foot is within bounds before the touchdown is triggered. This is the distance threshold
    * inside the foot for the CoP to trigger touchdown. In addition the ground reaction force needs to be above
    * the threshold defined in {@link #setDefaultContactThresholdForce(double)}
    */
   public void setDefaultCoPThresholdDistance(double defaultCoPThresholdDistance)
   {
      this.defaultCoPThresholdDistance = defaultCoPThresholdDistance;
   }

   /**
    * This threshold is a second boundary for the ground contact force required for the controller to
    * assume foot contact after a step. If the ground contact force in z goes above this threshold the
    * foot touchdown is triggered regardless of the position of the CoP within the foothold. See
    * {@link #setDefaultContactThresholdForce(double)} for the first threshold.
    */
   public void setDefaultSecondContactThresholdForceIgnoringCoP(double defaultSecondContactThresholdForceIgnoringCoP)
   {
      this.defaultSecondContactThresholdForceIgnoringCoP = defaultSecondContactThresholdForceIgnoringCoP;
   }

   @Override
   public FootSwitchInterface newFootSwitch(String namePrefix,
                                            ContactablePlaneBody foot,
                                            Collection<? extends ContactablePlaneBody> otherFeet,
                                            RigidBodyBasics rootBody,
                                            ForceSensorDataReadOnly footForceSensor,
                                            double totalRobotWeight,
                                            YoRegistry registry)
   {
      if (contactThresholdForceParameter == null)
      {
         contactThresholdForceParameter = new DoubleParameter("ContactThresholdForce", registry, defaultContactThresholdForce);
         copThresholdFractionParameter = new DoubleParameter("CoPThresholdFraction", registry, defaultCoPThresholdDistance);
         secondContactThresholdForceParameter = new DoubleParameter("SecondContactThresholdForce", registry, defaultSecondContactThresholdForceIgnoringCoP);
      }

      return new WrenchBasedFootSwitch(namePrefix,
                                       footForceSensor,
                                       totalRobotWeight,
                                       foot,
                                       contactThresholdForceParameter,
                                       secondContactThresholdForceParameter,
                                       copThresholdFractionParameter,
                                       registry);
   }
}

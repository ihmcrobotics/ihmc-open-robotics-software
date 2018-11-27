package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.Collection;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.ContactSensor;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class WrenchBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdForce = Double.NaN;
   private double defaultCoPThresholdFraction = Double.NaN;
   private double defaultSecondContactThresholdForceIgnoringCoP = Double.NaN;

   private DoubleProvider contactThresholdForceParameter;
   private DoubleProvider copThresholdFractionParameter;
   private DoubleProvider secondContactThresholdForceParameter;

   public WrenchBasedFootSwitchFactory()
   {
   }

   public void setDefaultContactThresholdForce(double defaultContactThresholdForce)
   {
      this.defaultContactThresholdForce = defaultContactThresholdForce;
   }

   public void setDefaultCoPThresholdFraction(double defaultCoPThresholdFraction)
   {
      this.defaultCoPThresholdFraction = defaultCoPThresholdFraction;
   }

   public void setDefaultSecondContactThresholdForceIgnoringCoP(double defaultSecondContactThresholdForceIgnoringCoP)
   {
      this.defaultSecondContactThresholdForceIgnoringCoP = defaultSecondContactThresholdForceIgnoringCoP;
   }

   @Override
   public FootSwitchInterface newFootSwitch(String namePrefix, ContactablePlaneBody foot, Collection<? extends ContactablePlaneBody> otherFeet,
                                            ForceSensorDataReadOnly footForceSensor, ContactSensor footContactSensor, double totalRobotWeight,
                                            YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      if (contactThresholdForceParameter == null)
      {
         contactThresholdForceParameter = new DoubleParameter("ContactThresholdForce", registry, defaultContactThresholdForce);
         copThresholdFractionParameter = new DoubleParameter("CoPThresholdFraction", registry, defaultCoPThresholdFraction);
         secondContactThresholdForceParameter = new DoubleParameter("SecondContactThresholdForce", registry, defaultSecondContactThresholdForceIgnoringCoP);
      }

      return new WrenchBasedFootSwitch(namePrefix, footForceSensor, totalRobotWeight, foot, contactThresholdForceParameter,
                                       secondContactThresholdForceParameter, copThresholdFractionParameter, yoGraphicsListRegistry, registry);
   }
}

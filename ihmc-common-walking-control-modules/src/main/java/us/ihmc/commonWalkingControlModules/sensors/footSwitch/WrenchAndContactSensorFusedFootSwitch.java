package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ContactSensor;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class WrenchAndContactSensorFusedFootSwitch implements FootSwitchInterface
{
   private final WrenchBasedFootSwitch wrenchBasedFootSwitch;
   private final ContactSensor contactSensor;
   private final YoVariableRegistry registry;
   private final YoBoolean inContact;

   public WrenchAndContactSensorFusedFootSwitch(String namePrefix, ForceSensorDataReadOnly forceSensorData, ContactSensor contactSensor,
                                                double robotTotalWeight, ContactablePlaneBody contactablePlaneBody, DoubleProvider contactThresholdForce,
                                                DoubleProvider secondContactThresholdForce, DoubleProvider copThresholdFraction,
                                                YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      if (!forceSensorData.getMeasurementLink().equals(contactSensor.getRigidBody()))
      {
         throw new RuntimeException("The force sensor and the contact sensor are not on the same link.");
      }

      this.wrenchBasedFootSwitch = new WrenchBasedFootSwitch(namePrefix + "WrenchBasedFootSwitch", forceSensorData, robotTotalWeight, contactablePlaneBody,
                                                             contactThresholdForce, secondContactThresholdForce, copThresholdFraction, yoGraphicsListRegistry,
                                                             registry);

      this.contactSensor = contactSensor;

      this.inContact = new YoBoolean(namePrefix + "InContact", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean hasFootHitGround()
   {
      inContact.set(wrenchBasedFootSwitch.hasFootHitGround() && contactSensor.isInContact());
      return inContact.getBooleanValue();
   }

   @Override
   public double computeFootLoadPercentage()
   {
      return wrenchBasedFootSwitch.computeFootLoadPercentage();
   }

   @Override
   public void computeAndPackCoP(FramePoint2D copToPack)
   {
      wrenchBasedFootSwitch.computeAndPackCoP(copToPack);
   }

   @Override
   public void updateCoP()
   {
      wrenchBasedFootSwitch.updateCoP();
   }

   @Override
   public void computeAndPackFootWrench(Wrench footWrenchToPack)
   {
      wrenchBasedFootSwitch.computeAndPackFootWrench(footWrenchToPack);
   }

   @Override
   public ReferenceFrame getMeasurementFrame()
   {
      return wrenchBasedFootSwitch.getMeasurementFrame();
   }

   @Override
   public void reset()
   {
      wrenchBasedFootSwitch.reset();
      contactSensor.reset();
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return wrenchBasedFootSwitch.getForceMagnitudePastThreshhold();
   }

   @Override
   @Deprecated
   public void setFootContactState(boolean hasFootHitGround)
   {
      
   }

   @Override
   public void trustFootSwitch(boolean trustFootSwitch)
   {

   }

}

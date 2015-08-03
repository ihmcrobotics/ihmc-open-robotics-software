package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.model.ContactSensor;
import us.ihmc.robotics.humanoidRobot.model.ForceSensorDataReadOnly;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class WrenchAndContactSensorFusedFootSwitch implements FootSwitchInterface
{
   private final WrenchBasedFootSwitch wrenchBasedFootSwitch;
   private final ContactSensor contactSensor;
   private final YoVariableRegistry registry;
   private final BooleanYoVariable inContact;
   
   public WrenchAndContactSensorFusedFootSwitch(String namePrefix, ForceSensorDataReadOnly forceSensorData, ContactSensor contactSensor, 
         double footSwitchCoPThresholdFraction, double robotTotalWeight, ContactablePlaneBody contactablePlaneBody,
         YoGraphicsListRegistry yoGraphicsListRegistry, double contactThresholdForce, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      
      if(!forceSensorData.getMeasurementLink().equals(contactSensor.getRigidBody()))
      {
         throw new RuntimeException("The force sensor and the contact sensor are not on the same link.");
      }
      
      this.wrenchBasedFootSwitch = new WrenchBasedFootSwitch(namePrefix + "WrenchBasedFootSwitch", forceSensorData, 
            footSwitchCoPThresholdFraction, robotTotalWeight, contactablePlaneBody,
            yoGraphicsListRegistry, contactThresholdForce, registry);
      
      this.contactSensor = contactSensor;
      
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      
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
   public void computeAndPackCoP(FramePoint2d copToPack)
   {
      wrenchBasedFootSwitch.computeAndPackCoP(copToPack);
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

}

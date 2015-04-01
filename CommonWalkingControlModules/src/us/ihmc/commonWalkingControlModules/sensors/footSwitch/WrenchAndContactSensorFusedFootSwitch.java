package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.utilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.utilities.humanoidRobot.model.ContactSensor;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorData;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;

public class WrenchAndContactSensorFusedFootSwitch implements FootSwitchInterface
{
   private final WrenchBasedFootSwitch wrenchBasedFootSwitch;
   private final ContactSensorBasedFootswitch contactSensorBasedFootSwitch;
   private final YoVariableRegistry registry;
   
   public WrenchAndContactSensorFusedFootSwitch(String namePrefix, ForceSensorData forceSensorData, ContactSensor contactSensor, 
         double footSwitchCoPThresholdFraction, double robotTotalWeight, ContactablePlaneBody contactablePlaneBody,
         YoGraphicsListRegistry yoGraphicsListRegistry, double contactThresholdForce, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      
      if(forceSensorData.getMeasurementLink() != contactSensor.getRigidBody())
      {
         throw new RuntimeException("The force sensor and the contact sensor are not on the same link.");
      }
      
      this.wrenchBasedFootSwitch = new WrenchBasedFootSwitch(namePrefix + "WrenchBasedFootSwitch", forceSensorData, 
            footSwitchCoPThresholdFraction, robotTotalWeight, contactablePlaneBody,
            yoGraphicsListRegistry, contactThresholdForce, registry);
      
      this.contactSensorBasedFootSwitch = new ContactSensorBasedFootswitch(namePrefix + "ContactSensorBasedFootSwitch", contactSensor, registry);
   }

   @Override
   public boolean hasFootHitGround()
   {  
      return wrenchBasedFootSwitch.hasFootHitGround() & contactSensorBasedFootSwitch.hasFootHitGround();
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
      contactSensorBasedFootSwitch.reset();
   }

   @Override
   public boolean getForceMagnitudePastThreshhold()
   {
      return wrenchBasedFootSwitch.getForceMagnitudePastThreshhold();
   }

}

package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;


/**
 * @author twan
 *         Date: 5/28/13
 */
public class DrivingPuppetMasterUsingChangedListeners
{
   private final DrivingInterface drivingInterface;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable desiredSteeringWheelAngle = new DoubleYoVariable("desiredSteeringWheelAngle", registry);
   private final DoubleYoVariable desiredGasPedalPosition = new DoubleYoVariable("desiredGasPedalZ", registry);
   private final DoubleYoVariable desiredBrakePedalPosition = new DoubleYoVariable("desiredBrakePedalZ", registry);
   private final BooleanYoVariable desiredHandBrakeMode = new BooleanYoVariable("desiredParkingBrakeMode", registry);
   private final EnumYoVariable<DrivingInterface.GearName> desiredGear = new EnumYoVariable<DrivingInterface.GearName>("desiredGearName", "", registry, DrivingInterface.GearName.class, true);

   public DrivingPuppetMasterUsingChangedListeners(YoVariableRegistry parentRegistry, DrivingInterface drivingInterface)
   {
      this.drivingInterface = drivingInterface;
      parentRegistry.addChild(registry);
      desiredGear.set(null);
      desiredHandBrakeMode.set(true);

      setUpChangedListeners();
   }

   private void setUpChangedListeners()
   {
      desiredSteeringWheelAngle.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            drivingInterface.turnSteeringWheel(desiredSteeringWheelAngle.getDoubleValue());
         }
      });

      desiredGasPedalPosition.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            drivingInterface.pressGasPedal(desiredGasPedalPosition.getDoubleValue());
         }
      });

      desiredBrakePedalPosition.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            drivingInterface.pressBrakePedal(desiredBrakePedalPosition.getDoubleValue());
         }
      });

      desiredHandBrakeMode.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            drivingInterface.setHandBrake(desiredHandBrakeMode.getBooleanValue(), false);
         }
      });

      desiredGear.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            drivingInterface.setGear(desiredGear.getEnumValue(), false);
         }
      });
   }


}

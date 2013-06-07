package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving;

import com.yobotics.simulationconstructionset.*;

/**
 * @author twan
 *         Date: 5/28/13
 */
public class DrivingPuppetMasterUsingChangedListeners
{
   private final DrivingInterface drivingInterface;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleYoVariable desiredSteeringWheelAngle = new DoubleYoVariable("desiredSteeringWheelAngle", registry);
   private final DoubleYoVariable desiredGasPedalPosition = new DoubleYoVariable("desiredGasPedalAngle", registry);
   private final DoubleYoVariable desiredBrakePedalPosition = new DoubleYoVariable("desiredBrakePedalAngle", registry);
   private final BooleanYoVariable desiredParkingBrakeMode = new BooleanYoVariable("desiredParkingBrakeMode", registry);
   private final EnumYoVariable<DrivingInterface.GearName> desiredGear = new EnumYoVariable<DrivingInterface.GearName>("desiredGearName", "", registry, DrivingInterface.GearName.class, true);

   public DrivingPuppetMasterUsingChangedListeners(YoVariableRegistry parentRegistry, DrivingInterface drivingInterface)
   {
      this.drivingInterface = drivingInterface;
      parentRegistry.addChild(registry);
      desiredGear.set(null);

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

      desiredParkingBrakeMode.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            drivingInterface.setParkingBrake(desiredParkingBrakeMode.getBooleanValue());
         }
      });

      desiredGear.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable v)
         {
            drivingInterface.setGear(desiredGear.getEnumValue());
         }
      });
   }


}

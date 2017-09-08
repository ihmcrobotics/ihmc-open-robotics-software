package us.ihmc.wanderer.hardware.state;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.acsell.hardware.state.AcsellActuatorState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.wanderer.hardware.WandererActuator;

public class WandererBatteryMonitor
{
   private static final WandererActuator voltageCurrentBatteryMonitorActuator = WandererActuator.RIGHT_HIP_X;
   private static final WandererActuator temperatureBatteryMonitorActuator = WandererActuator.RIGHT_HIP_Y;
   private static final int NUM_BATTERIES = 2;
   private static final int CURRENT_START_ID_2 = 2;
   
   private final EnumMap<WandererActuator, AcsellActuatorState> actuatorStates;
      
   private final YoVariableRegistry registry = new YoVariableRegistry("WandererBatteryMonitor");
   private final ArrayList<YoDouble> batteryVoltages = new ArrayList<YoDouble>();
   private final ArrayList<YoDouble>  batteryCurrents = new ArrayList<YoDouble>();
   private final ArrayList<YoDouble>  batteryTemperatures = new ArrayList<YoDouble>();
   private final YoDouble totalVoltage = new YoDouble("totalBatteryVoltage", registry);
   
   public WandererBatteryMonitor(EnumMap<WandererActuator, AcsellActuatorState> actuatorStates, YoVariableRegistry parentRegistry)
   {
      this.actuatorStates = actuatorStates;
      for(int i=0;i<NUM_BATTERIES; i++)
      {
         batteryVoltages.add(new YoDouble("battery" + i + "Voltage", registry));
         batteryCurrents.add(new YoDouble("battery" + i + "Current", registry));
         batteryTemperatures.add(new YoDouble("battery" + i + "Temperature", registry));
      }    
      
      parentRegistry.addChild(registry);
   }
   

   public void update()
   {
      for(int id = 0; id < NUM_BATTERIES; id++)
      {
         batteryVoltages.get(id).set(actuatorStates.get(voltageCurrentBatteryMonitorActuator).getPressureSensor(id).getRawValue()/100.0);
         batteryCurrents.get(id).set(actuatorStates.get(voltageCurrentBatteryMonitorActuator).getPressureSensor(id+CURRENT_START_ID_2).getRawValue()/100.0);
         batteryTemperatures.get(id).set(actuatorStates.get(temperatureBatteryMonitorActuator).getPressureSensor(id).getRawValue());
      }
      totalVoltage.set(batteryVoltages.get(0).getDoubleValue() + batteryVoltages.get(1).getDoubleValue()); 
   }

}

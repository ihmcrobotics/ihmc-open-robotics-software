package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving.DrivingInterface;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving.DrivingInterface.GearName;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.driving.VehicleModelObjects;
import us.ihmc.packets.LowLevelDrivingCommand;
import us.ihmc.utilities.net.ObjectConsumer;

public class DrivingCommandProvider implements ObjectConsumer<LowLevelDrivingCommand>
{
   private final ConcurrentLinkedQueue<LowLevelDrivingCommand> drivingCommands = new ConcurrentLinkedQueue<LowLevelDrivingCommand>();
   private DrivingInterface drivingInterface;
   
   private double maximumGasPedalDistance;
   private double maximumBrakePedalDistance;
   
   public void consumeObject(LowLevelDrivingCommand object)
   {
      drivingCommands.add(object);
   }
   
   private GearName getGearName(double value)
   {
      if(value > 0.0)
      {
         return GearName.FORWARD;
      }
      else
      {
         return GearName.REVERSE;
      }
      
   }
   
   private boolean getHandbrakeState(double value)
   {
      if(value > 0.0)
      {
         return true;
      }
      else
      {
         return false;
      }
      
   }

   public void doControl()
   {
      LowLevelDrivingCommand command = drivingCommands.poll();
      while(command != null)
      {
         double value = command.getValue();
         switch(command.getAction())
         {
         case DIRECTION:  
            drivingInterface.setGear(getGearName(value), false);
         break;
         case FOOTBRAKE: 
            drivingInterface.pressBrakePedal(value * maximumBrakePedalDistance);
         break;
         case DO_NOTHING:
            break;
         case GASPEDAL:
            drivingInterface.pressGasPedal(value * maximumGasPedalDistance);
            break;
         case GET_IN_CAR:
            System.err.println("Cannot get in the car using magic");
            break;
         case HANDBRAKE:
            drivingInterface.setHandBrake(getHandbrakeState(value), false);
            break;
         case STEERING:
            drivingInterface.turnSteeringWheel(value);
            break;
         }
         
         command = drivingCommands.poll();
      }
      
   }

   public void setDrivingInterfaceAndVehicleModel(DrivingInterface drivingInterface, VehicleModelObjects vehicleModelObjects)
   {
      this.drivingInterface = drivingInterface;
      this.maximumGasPedalDistance = vehicleModelObjects.getMaximumGasPedalDistance();
      this.maximumBrakePedalDistance = vehicleModelObjects.getMaximumBrakePedalDistance();
   }

}

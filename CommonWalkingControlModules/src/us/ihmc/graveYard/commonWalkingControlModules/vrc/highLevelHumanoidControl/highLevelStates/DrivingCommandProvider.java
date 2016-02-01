package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.highLevelStates;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.DrivingInterface;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.DrivingInterface.GearName;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingAction;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingCommand;
import us.ihmc.humanoidRobotics.communication.packets.LowLevelDrivingStatus;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.driving.VehicleModelObjects;

public class DrivingCommandProvider implements PacketConsumer<LowLevelDrivingCommand>
{
   private final ConcurrentLinkedQueue<LowLevelDrivingCommand> drivingCommands = new ConcurrentLinkedQueue<LowLevelDrivingCommand>();
   private DrivingInterface drivingInterface;
   
   private double maximumGasPedalDistance;
   private double maximumBrakePedalDistance;
   
   private boolean doNothing = false;
   private double doNothingEndTime = 0;
   private double clearanceFromPedals;


   public void receivedPacket(LowLevelDrivingCommand object)
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

   private boolean overrideChecks(double value)
   {
      if(value > 1.5 || value < -1.5)
      {
         return true;
      }
      else
      {
         return false;
      }
   }
   public void doControl(double currentTime)
   {
      LowLevelDrivingCommand command = drivingCommands.poll();
      while(command != null)
      {
         double value = command.getValue();
         switch(command.getAction())
         {
         case DIRECTION:  
            drivingInterface.setGear(getGearName(value), overrideChecks(value));
         break;
         case FOOTBRAKE:
            drivingInterface.pressBrakePedal(computePedalPosition(value, maximumBrakePedalDistance));
         break;
         case GASPEDAL:
            drivingInterface.pressGasPedal(computePedalPosition(value, maximumGasPedalDistance));
            break;
         case GET_IN_CAR:
            System.err.println("Cannot get in the car using magic");
            break;
         case HANDBRAKE:
            drivingInterface.setHandBrake(getHandbrakeState(value), overrideChecks(value));
            break;
         case STEERING:
            drivingInterface.turnSteeringWheel(value);
            break;
         case DO_NOTHING:
            doNothing = true;
            doNothingEndTime = currentTime + value;
            break;
         case REINITIALIZE:
            drivingInterface.reinitialize();
            break;
         }
         
         command = drivingCommands.poll();
      }
      
      
      if(doNothing)
      {
         if(currentTime > doNothingEndTime)
         {
            doNothing = false;
            drivingInterface.getStatusProducer().queueDataToSend(new LowLevelDrivingStatus(LowLevelDrivingAction.DO_NOTHING, true));
         }
      }
   }

   private double computePedalPosition(double value, double maximumPedalDistance)
   {
      double position = value * (maximumPedalDistance - clearanceFromPedals) + clearanceFromPedals;
//      if (Math.abs(position) < 1e-4)
//         return clearanceFromPedals;
//      else
//      {
//         return position;
//      }
      return position;
   }

   public void setDrivingInterfaceAndVehicleModel(DrivingInterface drivingInterface, VehicleModelObjects vehicleModelObjects, double clearanceFromPedals)
   {
      this.drivingInterface = drivingInterface;
      this.maximumGasPedalDistance = vehicleModelObjects.getMaximumGasPedalDistance();
      this.maximumBrakePedalDistance = vehicleModelObjects.getMaximumBrakePedalDistance();
      this.clearanceFromPedals = clearanceFromPedals;
   }

}

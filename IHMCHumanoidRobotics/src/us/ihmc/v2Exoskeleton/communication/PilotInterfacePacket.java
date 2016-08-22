package us.ihmc.v2Exoskeleton.communication;

import us.ihmc.communication.packets.StatusPacket;

/**
 * Created by shadylady on 8/9/16.
 */
public class PilotInterfacePacket extends StatusPacket<PilotInterfacePacket>
{
   public int behaviourState;
   public int requestedBehaviorState;
   public int desiredStepType;
   public int desiredStepLengthType;
   public int desiredStepRampType;
   public int desiredStepStairsType;
   public double controllerTime;
   public boolean desiredStepContinousWalk;
   public boolean requestStandUp;
   public int desiredStepsToTake;

   public PilotInterfacePacket()
   {
   }

   public PilotInterfacePacket(int behaviourState, int requestedBehaviorState, int desiredStepType, int desiredStepLengthType, int desiredStepRampType,
         int desiredStepStairsType, double controllerTime, boolean desiredStepContinousWalk, boolean requestStandUp, int desiredStepsToTake)
   {
      this.behaviourState = behaviourState;
      this.requestedBehaviorState = requestedBehaviorState;
      this.desiredStepType = desiredStepType;
      this.desiredStepLengthType = desiredStepLengthType;
      this.desiredStepRampType = desiredStepRampType;
      this.desiredStepStairsType = desiredStepStairsType;
      this.controllerTime = controllerTime;
      this.desiredStepContinousWalk = desiredStepContinousWalk;
      this.requestStandUp = requestStandUp;
      this.desiredStepsToTake = desiredStepsToTake;
   }

   public int getBehaviourState()
   {
      return behaviourState;
   }

   public int getRequestedBehaviourState()
   {
      return requestedBehaviorState;
   }

   public int getDesiredStepType()
   {
      return desiredStepType;
   }

   public int getDesiredStepLengthType()
   {
      return desiredStepLengthType;
   }

   public int getDesiredStepRampType()
   {
      return desiredStepRampType;
   }

   public int getDesiredStepStairsType()
   {
      return desiredStepStairsType;
   }

   public double getControllerTime()
   {
      return controllerTime;
   }

   public boolean getDesiredStepContinousWalk()
   {
      return desiredStepContinousWalk;
   }

   public boolean getRequestStandUp()
   {
      return requestStandUp;
   }

   public int getDesiredStepsToTake()
   {
      return desiredStepsToTake;
   }

   @Override public void set(PilotInterfacePacket other)
   {
      this.behaviourState = other.behaviourState;
      this.requestedBehaviorState = other.requestedBehaviorState;
      this.desiredStepType = other.desiredStepType;
      this.desiredStepLengthType = other.desiredStepLengthType;
      this.desiredStepRampType = other.desiredStepRampType;
      this.desiredStepStairsType = other.desiredStepStairsType;
      this.desiredStepContinousWalk = other.desiredStepContinousWalk;
      this.requestStandUp = other.requestStandUp;
      this.desiredStepsToTake = other.desiredStepsToTake;
   }

   @Override public boolean epsilonEquals(PilotInterfacePacket other, double epsilon)
   {
      return other.equals(this);
   }
}

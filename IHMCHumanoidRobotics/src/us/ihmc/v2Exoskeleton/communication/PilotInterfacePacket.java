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
   public int desiredStepStairsType;
   public boolean desiredStepContinousWalk;
   public int desiredStepsToTake;
   public boolean executeBehavior;

   public PilotInterfacePacket()
   {
   }

   public PilotInterfacePacket(int behaviourState, int requestedBehaviorState, int desiredStepType, int desiredStepLengthType, int desiredStepStairsType, boolean desiredStepContinousWalk,
                               int desiredStepsToTake, boolean desiredStepSend)
   {
      this.behaviourState = behaviourState;
      this.requestedBehaviorState = requestedBehaviorState;
      this.desiredStepType = desiredStepType;
      this.desiredStepLengthType = desiredStepLengthType;
      this.desiredStepStairsType = desiredStepStairsType;
      this.desiredStepContinousWalk = desiredStepContinousWalk;
      this.desiredStepsToTake = desiredStepsToTake;
      this.executeBehavior = desiredStepSend;
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

   public int getDesiredStepStairsType()
   {
      return desiredStepStairsType;
   }

   public boolean getDesiredStepContinousWalk()
   {
      return desiredStepContinousWalk;
   }

   public int getDesiredStepsToTake()
   {
      return desiredStepsToTake;
   }

   public boolean getExecuteBehavior()
   {
      return executeBehavior;
   }

   @Override
   public void set(PilotInterfacePacket other)
   {
      this.behaviourState = other.behaviourState;
      this.requestedBehaviorState = other.requestedBehaviorState;
      this.desiredStepType = other.desiredStepType;
      this.desiredStepLengthType = other.desiredStepLengthType;
      this.desiredStepStairsType = other.desiredStepStairsType;
      this.desiredStepContinousWalk = other.desiredStepContinousWalk;
      this.desiredStepsToTake = other.desiredStepsToTake;
      this.executeBehavior = other.executeBehavior;
   }

   public boolean arePacketsEqual(PilotInterfacePacket other)
   {
      return (this.behaviourState == other.getBehaviourState() && this.requestedBehaviorState == other.getRequestedBehaviourState() && this.desiredStepType == other.desiredStepType && this.desiredStepLengthType == other.desiredStepLengthType
            && this.desiredStepStairsType == other.desiredStepStairsType && this.desiredStepContinousWalk == other.desiredStepContinousWalk
            && this.desiredStepsToTake == other.desiredStepsToTake && this.executeBehavior
            == other.executeBehavior && this.uniqueId == other.uniqueId && this.destination == other.destination
            && this.notes == other.notes);
   }

   @Override
   public boolean epsilonEquals(PilotInterfacePacket other, double epsilon)
   {
      return other.equals(this);
   }

   @Override public String toString()
   {
      String packetString = new String("behaviorstate: " +  behaviourState + "\n" + " requested behaviorstate: " + requestedBehaviorState + "\n" + "desired steptype: " + desiredStepType + "\n"
                                             + "desired step length type: " + desiredStepLengthType + "\n" + "desired stairs step type: "  + desiredStepStairsType + "\n" + "desiredStepContinuouswalk: " + desiredStepContinousWalk
                                             + "\n" + "desired steps to take: " + desiredStepsToTake + "\n" + "execute behavior: " + executeBehavior);
      return packetString;
   }
}

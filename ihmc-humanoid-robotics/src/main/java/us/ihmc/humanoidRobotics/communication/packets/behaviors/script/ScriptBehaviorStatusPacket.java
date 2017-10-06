package us.ihmc.humanoidRobotics.communication.packets.behaviors.script;

import java.util.Random;

import us.ihmc.communication.packets.Packet;

public class ScriptBehaviorStatusPacket extends Packet<ScriptBehaviorStatusPacket>
{
   public ScriptBehaviorStatusEnum currentStatus;
   public int scriptIndex;

   public ScriptBehaviorStatusPacket()
   {
   }

   public ScriptBehaviorStatusPacket(ScriptBehaviorStatusEnum status)
   {
      this.currentStatus = status;
   }

   public ScriptBehaviorStatusPacket(ScriptBehaviorStatusEnum status, int scriptIndex)
   {
      this.currentStatus = status;
      this.scriptIndex = scriptIndex;
   }

   public ScriptBehaviorStatusEnum getScriptStatus()
   {
      return currentStatus;
   }

   public int getScriptCommandIndex()
   {
      return scriptIndex;
   }

   @Override
   public boolean epsilonEquals(ScriptBehaviorStatusPacket other, double epsilon)
   {
      boolean result = getScriptStatus().equals(other.getScriptStatus());
      result &= getScriptCommandIndex() == other.getScriptCommandIndex();

      return result;
   }

   public ScriptBehaviorStatusPacket(Random random)
   {
      int randomOrdinal = random.nextInt(ScriptBehaviorStatusEnum.values().length);
      ScriptBehaviorStatusEnum status = ScriptBehaviorStatusEnum.values()[randomOrdinal];
      int randomIndex = random.nextInt(255);

      this.currentStatus = status;
      this.scriptIndex = randomIndex;

   }
}

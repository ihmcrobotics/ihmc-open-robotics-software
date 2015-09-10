package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class ScriptTask extends BehaviorTask
{
   private final ScriptBehavior scriptBehavior;
   private final ScriptBehaviorInputPacket scriptBehaviorInputPacket;

   private final boolean DEBUG = false;

   public ScriptTask(ScriptBehavior scriptBehavior, ScriptBehaviorInputPacket scriptBehaviorInputPacket, DoubleYoVariable yoTime)
   {
      super(scriptBehavior, yoTime);
      this.scriptBehavior = scriptBehavior;
      this.scriptBehaviorInputPacket = scriptBehaviorInputPacket;
   }

   @Override
   protected void setBehaviorInput()
   {
      scriptBehavior.importScriptInputPacket(scriptBehaviorInputPacket);
   }
}

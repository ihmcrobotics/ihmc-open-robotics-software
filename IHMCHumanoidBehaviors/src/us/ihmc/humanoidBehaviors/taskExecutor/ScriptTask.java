package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.behaviors.script.ScriptBehaviorInputPacket;
import us.ihmc.humanoidBehaviors.behaviors.scripts.ScriptBehavior;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.taskExecutor.Task;

public class ScriptTask implements Task
{
   private final ScriptBehavior scriptBehavior;
   private final ScriptBehaviorInputPacket scriptBehaviorInputPacket;

   private final boolean DEBUG = false;

   public ScriptTask(ScriptBehavior scriptBehavior, ScriptBehaviorInputPacket scriptBehaviorInputPacket)
   {
      this.scriptBehavior = scriptBehavior;
      this.scriptBehaviorInputPacket = scriptBehaviorInputPacket;
   }

   @Override
   public void doTransitionIntoAction()
   {
      SysoutTool.println("entering scriptTask", DEBUG);
      scriptBehavior.initialize();
      scriptBehavior.importScriptInputPacket(scriptBehaviorInputPacket);
   }

   @Override
   public void doAction()
   {
      scriptBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      SysoutTool.println("exiting scriptTask", DEBUG);
      scriptBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return scriptBehavior.isDone();
   }

}

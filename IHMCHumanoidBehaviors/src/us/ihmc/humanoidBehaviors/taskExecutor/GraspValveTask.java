package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.SysoutTool;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.taskExecutor.Task;

public class GraspValveTask implements Task
{
   private final GraspValveBehavior graspValveBehavior;

   public GraspValveTask(GraspValveBehavior graspValveBehavior, ValveType valveType, RigidBodyTransform valveTransformToWorld,
         Vector3d approachDirection, boolean graspValveRim)
   {
      this.graspValveBehavior = graspValveBehavior;
      
      graspValveBehavior.initialize();
      graspValveBehavior.setGraspPose(valveType, valveTransformToWorld, approachDirection, graspValveRim);
   }

   @Override
   public void doTransitionIntoAction()
   {
      SysoutTool.println("entering graspValveBehavior");
   }

   @Override
   public void doAction()
   {
      graspValveBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      System.out.println("exiting walkToLocationTask");
      graspValveBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return graspValveBehavior.isDone();
   }

}

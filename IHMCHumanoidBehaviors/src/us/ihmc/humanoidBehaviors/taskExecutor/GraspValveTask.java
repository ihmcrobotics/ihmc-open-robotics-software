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
   private final ValveType valveType;
   private final RigidBodyTransform valveTransformToWorld;
   private final Vector3d approachDirection;
   private final boolean graspValveRim;

   private final boolean DEBUG = false;
   
   public GraspValveTask(GraspValveBehavior graspValveBehavior, ValveType valveType, RigidBodyTransform valveTransformToWorld,
         Vector3d approachDirection, boolean graspValveRim)
   {
      this.graspValveBehavior = graspValveBehavior;
      this.valveType = valveType;
      this.valveTransformToWorld = valveTransformToWorld;
      this.approachDirection = approachDirection;
      this.graspValveRim = graspValveRim;
      
      graspValveBehavior.initialize();
      graspValveBehavior.setGraspPose(valveType, valveTransformToWorld, approachDirection, graspValveRim);  // Must do this here, otherwise grasp pose may not be computed before other behaviors reference it
   }

   @Override
   public void doTransitionIntoAction()
   {
//      graspValveBehavior.initialize();
//      graspValveBehavior.setGraspPose(valveType, valveTransformToWorld, approachDirection, graspValveRim);  //FIXME:  For some reason this causes next handPose to be start of approach grasp vector
      SysoutTool.println("entering graspValveTask", DEBUG);
   }

   @Override
   public void doAction()
   {
      graspValveBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      SysoutTool.println("exiting graspValveTask", DEBUG);
      graspValveBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return graspValveBehavior.isDone();
   }

}

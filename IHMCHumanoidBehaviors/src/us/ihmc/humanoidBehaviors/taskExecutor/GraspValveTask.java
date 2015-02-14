package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.GraspValveBehavior;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveTask extends BehaviorTask
{
   private final GraspValveBehavior graspValveBehavior;
   private final ValveType valveType;
   private final RigidBodyTransform valveTransformToWorld;
   private final Vector3d approachDirection;
   private final boolean graspValveRim;

   private final boolean DEBUG = false;

   public GraspValveTask(GraspValveBehavior graspValveBehavior, ValveType valveType, RigidBodyTransform valveTransformToWorld, Vector3d approachDirection,
         boolean graspValveRim, DoubleYoVariable yoTime)
   {
      super(graspValveBehavior, yoTime);
      this.graspValveBehavior = graspValveBehavior;
      this.valveType = valveType;
      this.valveTransformToWorld = new RigidBodyTransform(valveTransformToWorld);
      this.approachDirection = new Vector3d(approachDirection);
      this.graspValveRim = graspValveRim;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspValveBehavior.setGraspPose(valveType, valveTransformToWorld, approachDirection, graspValveRim);
   }
}

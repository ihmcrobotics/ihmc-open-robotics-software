package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Vector3d;

import us.ihmc.humanoidBehaviors.behaviors.GraspValveTurnAndUnGraspBehavior;
import us.ihmc.simulationconstructionset.util.environments.ValveType;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class GraspValveTurnAndUnGraspTask extends BehaviorTask
{
   private final GraspValveTurnAndUnGraspBehavior graspValveTurnAndUnGraspBehavior;
   private final ValveType valveType;
   private final RigidBodyTransform valveTransformToWorld;
   private final Vector3d graspApproachDirectionInValveFrame;
   private final Axis valvePinJointAxisInValveFrame;
   private final boolean graspValveRim;
   private final double turnValveAngle;
   
   private final boolean DEBUG = false;

   public GraspValveTurnAndUnGraspTask(GraspValveTurnAndUnGraspBehavior graspValveTurnAndUnGraspBehavior, ValveType valveType,
         RigidBodyTransform valveTransformToWorld, Vector3d graspApproachDirectionInValveFrame, Axis valvePinJointAxisInValveFrame, boolean graspValveRim, double turnValveAngle, DoubleYoVariable yoTime)
   {
      super(graspValveTurnAndUnGraspBehavior, yoTime);
      this.graspValveTurnAndUnGraspBehavior = graspValveTurnAndUnGraspBehavior;
      this.valveType = valveType;
      this.valveTransformToWorld = new RigidBodyTransform(valveTransformToWorld);
      this.graspApproachDirectionInValveFrame = new Vector3d(graspApproachDirectionInValveFrame);
      this.valvePinJointAxisInValveFrame = valvePinJointAxisInValveFrame;
      this.graspValveRim = graspValveRim;
      this.turnValveAngle = turnValveAngle;
   }

   @Override
   protected void setBehaviorInput()
   {
      graspValveTurnAndUnGraspBehavior.setInput(valveType, valveTransformToWorld, graspApproachDirectionInValveFrame, valvePinJointAxisInValveFrame, graspValveRim, turnValveAngle);
   }
}

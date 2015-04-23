package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.stateMachines.State;


public abstract class TaskspaceHandControlState extends State<HandControlState>
{
   protected final String name;
   protected final YoVariableRegistry registry;
   protected final int jacobianId;

   private RigidBody base;
   private RigidBody endEffector;
   
   protected final TaskspaceConstraintData taskspaceConstraintData = new TaskspaceConstraintData();
   protected final MomentumBasedController momentumBasedController;

   public TaskspaceHandControlState(String namePrefix, HandControlState stateEnum, MomentumBasedController momentumBasedController, int jacobianId,
                                    RigidBody base, RigidBody endEffector, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      name = namePrefix + FormattingTools.underscoredToCamelCase(this.getStateEnum().toString(), true) + "State";
      registry = new YoVariableRegistry(name);

      taskspaceConstraintData.set(base, endEffector);

      this.momentumBasedController = momentumBasedController;
      this.jacobianId = jacobianId;
      this.base = base;

      parentRegistry.addChild(registry);
   }

   protected void setBase(RigidBody newBase)
   {
      this.base = newBase;
      taskspaceConstraintData.set(base, endEffector);
   }

   protected void setEndEffector(RigidBody newEndEffector)
   {
      this.endEffector = newEndEffector;
      taskspaceConstraintData.set(base, endEffector);
   }

   protected RigidBody getBase()
   {
      return base;
   }

   protected RigidBody getEndEffector()
   {
      return endEffector;
   }

   @Override
   public void doAction()
   {
      SpatialAccelerationVector handAcceleration = computeDesiredSpatialAcceleration();
      taskspaceConstraintData.set(handAcceleration);
      momentumBasedController.setDesiredSpatialAcceleration(jacobianId, taskspaceConstraintData);
   }

   protected abstract SpatialAccelerationVector computeDesiredSpatialAcceleration();

   @Override
   public void doTransitionIntoAction()
   {
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }
}

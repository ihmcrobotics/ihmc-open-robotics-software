package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ControlledBodiesCommand implements InverseDynamicsCommand<ControlledBodiesCommand>
{
   private List<RigidBody> controlledBodies = new ArrayList<>();
   private List<RigidBody> basesForControl = new ArrayList<>();
   private List<OneDoFJoint[]> jointsToUse = new ArrayList<>();

   public ControlledBodiesCommand()
   {
   }

   @Override
   public void set(ControlledBodiesCommand other)
   {
      this.controlledBodies.clear();
      this.basesForControl.clear();
      this.jointsToUse.clear();

      for (int i = 0; i < other.getNumberOfControlledBodies(); i++)
      {
         this.controlledBodies.set(i, other.getControlledBody(i));
         this.basesForControl.set(i, other.getBaseForControl(i));
         this.jointsToUse.set(i, other.getJointsToUse(i));
      }
   }

   public void addBodyToControl(RigidBody controlledBody)
   {
      addBodyToControl(controlledBody, null, null);
   }

   public void addBodyToControl(RigidBody controlledBody, RigidBody baseForControl)
   {
      addBodyToControl(controlledBody, baseForControl, null);
   }

   public void addBodyToControl(RigidBody controlledBody, OneDoFJoint[] jointsToUse)
   {
      addBodyToControl(controlledBody, null, jointsToUse);
   }

   public void addBodyToControl(RigidBody controlledBody, RigidBody baseForControl, OneDoFJoint[] jointsToUse)
   {
      controlledBodies.add(controlledBody);
      basesForControl.add(baseForControl);
      this.jointsToUse.add(jointsToUse);
   }

   public int getNumberOfControlledBodies()
   {
      return controlledBodies.size();
   }

   public RigidBody getControlledBody(int i)
   {
      return controlledBodies.get(i);
   }

   public RigidBody getBaseForControl(int i)
   {
      return basesForControl.get(i);
   }

   public OneDoFJoint[] getJointsToUse(int i)
   {
      return jointsToUse.get(i);
   }

   public List<RigidBody> getAllControlledBodies()
   {
      return controlledBodies;
   }

   public List<RigidBody> getAllBasesForControl()
   {
      return basesForControl;
   }

   public List<OneDoFJoint[]> getAllJointsToUse()
   {
      return jointsToUse;
   }

   public boolean hasBaseForControl(int i)
   {
      return basesForControl.get(i) != null;
   }

   public boolean hasJointsToUse(int i)
   {
      return jointsToUse.get(i) != null;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.CONTROLLED_BODIES;
   }
}

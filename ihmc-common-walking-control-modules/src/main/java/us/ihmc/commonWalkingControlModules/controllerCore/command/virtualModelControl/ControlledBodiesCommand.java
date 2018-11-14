package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class ControlledBodiesCommand implements VirtualModelControlCommand<ControlledBodiesCommand>
{
   private List<RigidBodyBasics> controlledBodies = new ArrayList<>();
   private List<RigidBodyBasics> basesForControl = new ArrayList<>();
   private List<OneDoFJointBasics[]> jointsToUse = new ArrayList<>();

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

   public void addBodyToControl(RigidBodyBasics controlledBody)
   {
      addBodyToControl(controlledBody, null, null);
   }

   public void addBodyToControl(RigidBodyBasics controlledBody, RigidBodyBasics baseForControl)
   {
      addBodyToControl(controlledBody, baseForControl, null);
   }

   public void addBodyToControl(RigidBodyBasics controlledBody, OneDoFJointBasics[] jointsToUse)
   {
      addBodyToControl(controlledBody, null, jointsToUse);
   }

   public void addBodyToControl(RigidBodyBasics controlledBody, RigidBodyBasics baseForControl, OneDoFJointBasics[] jointsToUse)
   {
      controlledBodies.add(controlledBody);
      basesForControl.add(baseForControl);
      this.jointsToUse.add(jointsToUse);
   }

   public int getNumberOfControlledBodies()
   {
      return controlledBodies.size();
   }

   public RigidBodyBasics getControlledBody(int i)
   {
      return controlledBodies.get(i);
   }

   public RigidBodyBasics getBaseForControl(int i)
   {
      return basesForControl.get(i);
   }

   public OneDoFJointBasics[] getJointsToUse(int i)
   {
      return jointsToUse.get(i);
   }

   public List<RigidBodyBasics> getAllControlledBodies()
   {
      return controlledBodies;
   }

   public List<RigidBodyBasics> getAllBasesForControl()
   {
      return basesForControl;
   }

   public List<OneDoFJointBasics[]> getAllJointsToUse()
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

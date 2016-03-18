package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointAccelerationIntegrationCommand implements InverseDynamicsCommand<JointAccelerationIntegrationCommand>
{
   private final int initialCapacity = 15;
   private final List<String> jointNamesToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);

   public JointAccelerationIntegrationCommand()
   {
   }

   public void clear()
   {
      jointNamesToComputeDesiredPositionFor.clear();
      jointsToComputeDesiredPositionFor.clear();
   }

   public void addJointToComputeDesiredPositionFor(OneDoFJoint joint)
   {
      jointNamesToComputeDesiredPositionFor.add(joint.getName());
      jointsToComputeDesiredPositionFor.add(joint);
   }

   @Override
   public void set(JointAccelerationIntegrationCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJointsToComputeDesiedPositionFor(); i++)
      {
         jointNamesToComputeDesiredPositionFor.add(other.jointNamesToComputeDesiredPositionFor.get(i));
         jointsToComputeDesiredPositionFor.add(other.jointsToComputeDesiredPositionFor.get(i));
      }
   }

   public void retrieveJointsFromName(Map<String, ? extends OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < getNumberOfJointsToComputeDesiedPositionFor(); i++)
      {
         jointsToComputeDesiredPositionFor.set(i, nameToJointMap.get(jointNamesToComputeDesiredPositionFor.get(i)));
      }
   }

   public OneDoFJoint getJointToComputeDesiedPositionFor(int jointIndex)
   {
      return jointsToComputeDesiredPositionFor.get(jointIndex);
   }

   public int getNumberOfJointsToComputeDesiedPositionFor()
   {
      return jointsToComputeDesiredPositionFor.size();
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINT_ACCELERATION_INTEGRATION;
   }
}

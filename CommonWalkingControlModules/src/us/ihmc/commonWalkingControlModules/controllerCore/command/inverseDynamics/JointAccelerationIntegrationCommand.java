package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointAccelerationIntegrationCommand implements InverseDynamicsCommand<JointAccelerationIntegrationCommand>
{
   private final int initialCapacity = 15;
   private final List<String> jointNamesToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);
   private final List<OneDoFJoint> jointsToComputeDesiredPositionFor = new ArrayList<>(initialCapacity);
   private final TDoubleArrayList jointAlphaPosition = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList jointAlphaVelocity = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList jointMaxPositionError = new TDoubleArrayList(initialCapacity);
   private final TDoubleArrayList jointMaxVelocity = new TDoubleArrayList(initialCapacity);

   public JointAccelerationIntegrationCommand()
   {
   }

   public void clear()
   {
      jointNamesToComputeDesiredPositionFor.clear();
      jointsToComputeDesiredPositionFor.clear();
      jointAlphaPosition.reset();
      jointAlphaVelocity.reset();
      jointMaxPositionError.reset();
      jointMaxVelocity.reset();
   }

   public void addJointToComputeDesiredPositionFor(OneDoFJoint joint)
   {
      jointNamesToComputeDesiredPositionFor.add(joint.getName());
      jointsToComputeDesiredPositionFor.add(joint);
      jointAlphaPosition.add(Double.NaN);
      jointAlphaVelocity.add(Double.NaN);
      jointMaxPositionError.add(Double.NaN);
      jointMaxVelocity.add(Double.NaN);
   }

   public void setJointAlphas(int jointIndex, double alphaPosition, double alphaVelocity)
   {
      jointAlphaPosition.set(jointIndex, alphaPosition);
      jointAlphaVelocity.set(jointIndex, alphaVelocity);
   }

   public void setJointMaxima(int jointIndex, double maxPositionError, double maxVelocity)
   {
      jointMaxPositionError.set(jointIndex, maxPositionError);
      jointMaxVelocity.set(jointIndex, maxVelocity);
   }

   @Override
   public void set(JointAccelerationIntegrationCommand other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         jointNamesToComputeDesiredPositionFor.add(other.jointNamesToComputeDesiredPositionFor.get(i));
         jointsToComputeDesiredPositionFor.add(other.jointsToComputeDesiredPositionFor.get(i));
      }
   }

   public void retrieveJointsFromName(Map<String, ? extends OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < getNumberOfJointsToComputeDesiredPositionFor(); i++)
      {
         jointsToComputeDesiredPositionFor.set(i, nameToJointMap.get(jointNamesToComputeDesiredPositionFor.get(i)));
      }
   }

   public OneDoFJoint getJointToComputeDesiredPositionFor(int jointIndex)
   {
      return jointsToComputeDesiredPositionFor.get(jointIndex);
   }

   public double getJointAlphaPosition(int jointIndex)
   {
      return jointAlphaPosition.get(jointIndex);
   }

   public double getJointAlphaVelocity(int jointIndex)
   {
      return jointAlphaVelocity.get(jointIndex);
   }

   public double getJointMaxPositionError(int jointIndex)
   {
      return jointMaxPositionError.get(jointIndex);
   }

   public double getJointMaxVelocity(int jointIndex)
   {
      return jointMaxVelocity.get(jointIndex);
   }

   public int getNumberOfJointsToComputeDesiredPositionFor()
   {
      return jointsToComputeDesiredPositionFor.size();
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.JOINT_ACCELERATION_INTEGRATION;
   }
}

package us.ihmc.stateEstimation.ekf;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

public class RobotState extends ComposedState implements RobotStateIndexProvider
{
   public static final double GRAVITY = 9.81;

   private final boolean isFloating;

   private final Map<String, JointState> jointStatesByName = new HashMap<>();
   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();

   public RobotState(PoseState poseState, List<JointState> jointStates)
   {
      isFloating = poseState != null;
      if (isFloating)
      {
         addState(poseState);
      }

      for (JointState jointState : jointStates)
      {
         MutableInt jointStateStartIndex = new MutableInt(getSize());
         addState(jointState);
         String jointName = jointState.getJointName();
         jointStatesByName.put(jointName, jointState);
         jointIndecesByName.put(jointName, jointStateStartIndex);
      }
   }

   public JointState getJointState(String jointName)
   {
      return jointStatesByName.get(jointName);
   }

   @Override
   public int findJointPositionIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue();
   }

   @Override
   public int findJointVelocityIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue() + 1;
   }

   @Override
   public int findJointAccelerationIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue() + 2;
   }

   @Override
   public boolean isFloating()
   {
      return isFloating;
   }

   @Override
   public int findOrientationIndex()
   {
      checkFloating();
      return PoseState.orientationStart;
   }

   @Override
   public int findAngularVelocityIndex()
   {
      checkFloating();
      return PoseState.angularVelocityStart;
   }

   @Override
   public int findAngularAccelerationIndex()
   {
      checkFloating();
      return PoseState.angularAccelerationStart;
   }

   @Override
   public int findPositionIndex()
   {
      checkFloating();
      return PoseState.positionStart;
   }

   @Override
   public int findLinearVelocityIndex()
   {
      checkFloating();
      return PoseState.linearVelocityStart;
   }

   @Override
   public int findLinearAccelerationIndex()
   {
      checkFloating();
      return PoseState.linearAccelerationStart;
   }

   public double getGravity()
   {
      return GRAVITY;
   }

   private void checkFloating()
   {
      if (!isFloating)
      {
         throw new RuntimeException("Robot is not a floating base robot. Can not get pose indices.");
      }
   }
}

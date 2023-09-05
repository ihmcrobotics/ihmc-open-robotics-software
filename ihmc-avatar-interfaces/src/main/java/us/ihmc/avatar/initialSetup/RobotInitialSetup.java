package us.ihmc.avatar.initialSetup;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.simulationconstructionset.Robot;

public interface RobotInitialSetup<T extends Robot>
{
   void initializeRobot(T robot);

   void initializeFullRobotModel(FullHumanoidRobotModel fullRobotModel);

   void initializeRobot(RigidBodyBasics rootBody);

   void initializeRobotDefinition(RobotDefinition robotDefinition);

   void setInitialYaw(double yaw);

   double getInitialYaw();

   void setInitialGroundHeight(double groundHeight);

   double getInitialGroundHeight();

   void setOffset(Tuple3DReadOnly additionalOffset);

   Tuple3DReadOnly getOffset();

   /**
    * Indicates whether the robot can be reset to its initial sim configuration with the implementation
    * of this interface setup after the first time. This is deprecated since all initial setups should
    * support this eventually. An example of an initial setup that can be reused is the Atlas one.
    *
    * @return whether this initial setup will work more then once.
    */
   @Deprecated
   default boolean supportsReset()
   {
      return false;
   }
}

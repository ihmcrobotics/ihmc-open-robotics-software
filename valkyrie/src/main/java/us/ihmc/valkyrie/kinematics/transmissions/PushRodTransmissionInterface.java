package us.ihmc.valkyrie.kinematics.transmissions;

import us.ihmc.valkyrie.kinematics.LinearActuator;
import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;

public interface PushRodTransmissionInterface
{
   public abstract void actuatorToJointEffort(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void actuatorToJointVelocity(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void actuatorToJointPosition(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void jointToActuatorEffort(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void jointToActuatorVelocity(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void jointToActuatorPosition(LinearActuator[] act_data, ValkyrieJointInterface[] jnt_data);
}
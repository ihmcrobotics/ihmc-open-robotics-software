package us.ihmc.valkyrie.kinematics.transmissions;

import us.ihmc.valkyrie.kinematics.ValkyrieJointInterface;
import us.ihmc.valkyrie.roboNet.TurboDriver;

public interface PushRodTransmissionInterface
{

   public abstract void actuatorToJointEffort(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void actuatorToJointVelocity(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void actuatorToJointPosition(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void jointToActuatorEffort(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void jointToActuatorVelocity(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data);

   public abstract void jointToActuatorPosition(TurboDriver[] act_data, ValkyrieJointInterface[] jnt_data);

}
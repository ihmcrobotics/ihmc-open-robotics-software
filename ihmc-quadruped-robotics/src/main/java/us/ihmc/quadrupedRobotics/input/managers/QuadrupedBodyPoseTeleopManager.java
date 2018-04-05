package us.ihmc.quadrupedRobotics.input.managers;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.QuadrupedBodyOrientationMessage;
import controller_msgs.msg.dds.SO3TrajectoryPointMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;

public class QuadrupedBodyPoseTeleopManager
{
   private final PacketCommunicator packetCommunicator;

   private final AtomicDouble desiredCoMHeight = new AtomicDouble();
   private final AtomicDouble desiredOrientationYaw = new AtomicDouble();
   private final AtomicDouble desiredOrientationPitch = new AtomicDouble();
   private final AtomicDouble desiredOrientationRoll = new AtomicDouble();
   private final AtomicDouble desiredOrientationTime = new AtomicDouble();

   private final ComPositionPacket comPositionPacket = new ComPositionPacket();
   private final QuadrupedBodyOrientationMessage bodyOrientationMessage = new QuadrupedBodyOrientationMessage();

   public QuadrupedBodyPoseTeleopManager(double initialCoMHeight, PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
      desiredCoMHeight.set(initialCoMHeight);

      bodyOrientationMessage.setIsExpressedInAbsoluteTime(false);
   }

   public void setDesiredCoMHeight(double desiredCoMHeight)
   {
      this.desiredCoMHeight.set(desiredCoMHeight);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   public void update()
   {
      double comHeight = desiredCoMHeight.getAndSet(Double.NaN);
      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      if(!Double.isNaN(comHeight))
      {
         comPositionPacket.position.set(0.0, 0.0, comHeight);
         packetCommunicator.send(comPositionPacket);
      }

      if(!Double.isNaN(desiredYaw))
      {
         bodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().clear();
         SO3TrajectoryPointMessage trajectoryPointMessage = bodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         trajectoryPointMessage.setTime(desiredTime);
         packetCommunicator.send(bodyOrientationMessage);
      }
   }
}

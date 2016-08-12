package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.IMUPacket;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

/**
 * Class provides head IMU updates to the state estimator. Since the head IMU updates on Atlas
 * are coming from ROS they have to be sent to the estimator thread as a packet. In that case
 * the class is used as a packet consumer.
 *
 * If the simulation is started without network (e.g. unit tests) this can be used as robot
 * controller providing the head IMU measurements from simulation directly. This class is not
 * meant to be used as PacketConsumer and RobotController at the same time!
 *
 * @author Georg
 *
 */
public class HeadIMUSubscriber extends SimpleRobotController implements PacketConsumer<IMUPacket>
{
   private final IMUDefinition imuDefinition;
   private final AtomicReference<IMUPacket> packet = new AtomicReference<IMUPacket>(null);

   private final IMUMount imuMount;
   private final DoubleYoVariable robotTime;
   private final IMUPacket tempPacket = new IMUPacket();

   private final Vector3d linearAcceleration = new Vector3d();
   private final Quat4d orientation = new Quat4d();
   private final Vector3d angularVelocity = new Vector3d();

   public HeadIMUSubscriber(IMUDefinition imuDefinition, IMUMount imuMount, DoubleYoVariable time)
   {
      this.imuDefinition = imuDefinition;
      this.imuMount = imuMount;
      this.robotTime = time;
   }

   @Override
   public void receivedPacket(IMUPacket packet)
   {
      this.packet.set(packet);
   }

   public boolean getPacket(IMUPacket dataToPack)
   {
      IMUPacket packet = this.packet.getAndSet(null);

      if (packet == null)
         return false;

      dataToPack.angularVelocity.set(packet.angularVelocity);
      dataToPack.linearAcceleration.set(packet.linearAcceleration);
      dataToPack.orientation.set(packet.orientation);
      dataToPack.time = packet.time;
      return true;
   }

   public IMUDefinition getImuDefinition()
   {
      return imuDefinition;
   }

   @Override
   public void doControl()
   {
      if (imuMount == null)
         throw new RuntimeException("This should not be used as a robot controller if there is no imu mount.");

      imuMount.getLinearAccelerationInBody(linearAcceleration);
      imuMount.getOrientation(orientation);
      imuMount.getAngularVelocityInBody(angularVelocity);

      tempPacket.linearAcceleration.set(linearAcceleration);
      tempPacket.orientation.set(orientation);
      tempPacket.angularVelocity.set(angularVelocity);
      tempPacket.time = robotTime.getDoubleValue();

      this.packet.set(tempPacket);
   }

}

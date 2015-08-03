package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Vector3d;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.HandComplianceControlParametersPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class HandComplianceControlParametersProvider implements PacketConsumer<HandComplianceControlParametersPacket>
{
   public final SideDependentList<AtomicBoolean> hasNewRequest = new SideDependentList<AtomicBoolean>(new AtomicBoolean(false), new AtomicBoolean(false));
   public final SideDependentList<AtomicBoolean> isResetRequested = new SideDependentList<AtomicBoolean>(new AtomicBoolean(false), new AtomicBoolean(false));
   public final SideDependentList<AtomicReference<boolean[]>> enableLinearCompliance = new SideDependentList<AtomicReference<boolean[]>>(new AtomicReference<boolean[]>(null), new AtomicReference<boolean[]>(null));
   public final SideDependentList<AtomicReference<boolean[]>> enableAngularCompliance = new SideDependentList<AtomicReference<boolean[]>>(new AtomicReference<boolean[]>(null), new AtomicReference<boolean[]>(null));
   public final SideDependentList<AtomicReference<Vector3d>> desiredForce = new SideDependentList<AtomicReference<Vector3d>>(new AtomicReference<Vector3d>(null), new AtomicReference<Vector3d>(null));
   public final SideDependentList<AtomicReference<Vector3d>> desiredTorque = new SideDependentList<AtomicReference<Vector3d>>(new AtomicReference<Vector3d>(null), new AtomicReference<Vector3d>(null));
   public final SideDependentList<AtomicDouble> forceDeadzoneSize = new SideDependentList<AtomicDouble>(new AtomicDouble(Double.NaN), new AtomicDouble(Double.NaN));
   public final SideDependentList<AtomicDouble> torqueDeadzoneSize = new SideDependentList<AtomicDouble>(new AtomicDouble(Double.NaN), new AtomicDouble(Double.NaN));

   public HandComplianceControlParametersProvider()
   {
   }

   public boolean checkForNewRequest(RobotSide robotSide)
   {
      return hasNewRequest.get(robotSide).getAndSet(false);
   }

   public boolean isResetRequested(RobotSide robotSide)
   {
      return isResetRequested.get(robotSide).getAndSet(false);
   }

   public boolean[] getEnableLinearCompliance(RobotSide robotSide)
   {
      return enableLinearCompliance.get(robotSide).getAndSet(null);
   }

   public boolean[] getEnableAngularCompliance(RobotSide robotSide)
   {
      return enableAngularCompliance.get(robotSide).getAndSet(null);
   }

   public Vector3d getDesiredForce(RobotSide robotSide)
   {
      return desiredForce.get(robotSide).getAndSet(null);
   }

   public Vector3d getDesiredTorque(RobotSide robotSide)
   {
      return desiredTorque.get(robotSide).getAndSet(null);
   }

   public double getForceDeadzone(RobotSide robotSide)
   {
      return forceDeadzoneSize.get(robotSide).getAndSet(Double.NaN);
   }

   public double getTorqueDeadzone(RobotSide robotSide)
   {
      return torqueDeadzoneSize.get(robotSide).getAndSet(Double.NaN);
   }

   @Override
   public void receivedPacket(HandComplianceControlParametersPacket packet)
   {
      if (packet == null || packet.getRobotSide() == null)
         return;

      RobotSide robotSide = packet.getRobotSide();

      hasNewRequest.get(robotSide).set(true);

      if (packet.isEmpty())
      {
         isResetRequested.get(robotSide).set(true);
         return;
      }

      isResetRequested.get(robotSide).set(false);

      enableLinearCompliance.get(robotSide).set(packet.getEnableLinearCompliance());
      enableAngularCompliance.get(robotSide).set(packet.getEnableAngularCompliance());

      desiredForce.get(robotSide).set(new Vector3d(packet.getDesiredForce()));
      desiredTorque.get(robotSide).set(new Vector3d(packet.getDesiredTorque()));

      if (packet.getWrenchDeadzones() == null)
      {
         forceDeadzoneSize.get(robotSide).set(Double.NaN);
         torqueDeadzoneSize.get(robotSide).set(Double.NaN);
      }
      else
      {
         forceDeadzoneSize.get(robotSide).set(packet.getWrenchDeadzones()[0]);
         torqueDeadzoneSize.get(robotSide).set(packet.getWrenchDeadzones()[1]);
      }
   }
}

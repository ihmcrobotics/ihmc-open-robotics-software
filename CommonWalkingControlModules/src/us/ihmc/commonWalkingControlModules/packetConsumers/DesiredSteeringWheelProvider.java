package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.manipulation.DesiredSteeringAnglePacket;
import us.ihmc.communication.packets.manipulation.SteeringWheelInformationPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import com.google.common.util.concurrent.AtomicDouble;

public class DesiredSteeringWheelProvider implements PacketConsumer<SteeringWheelInformationPacket>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SideDependentList<AtomicReference<Point3d>> steeringWheelCenterAtomic = new SideDependentList<>(new AtomicReference<Point3d>(null), new AtomicReference<Point3d>(null));
   private final SideDependentList<AtomicReference<Vector3d>> steeringWheelRotationAxisAtomic = new SideDependentList<>(new AtomicReference<Vector3d>(null), new AtomicReference<Vector3d>(null));
   private final SideDependentList<AtomicReference<Vector3d>> steeringWheelZeroAxisAtomic = new SideDependentList<>(new AtomicReference<Vector3d>(null), new AtomicReference<Vector3d>(null));
   private final SideDependentList<AtomicDouble> steeringWheelRadiusAtomic = new SideDependentList<>(new AtomicDouble(Double.NaN), new AtomicDouble(Double.NaN));
   private final SideDependentList<AtomicDouble> graspOffsetFromCotnrolFrameAtomic = new SideDependentList<>(new AtomicDouble(Double.NaN), new AtomicDouble(Double.NaN));
   private final SideDependentList<AtomicInteger> steeringWheelIdAtomic = new SideDependentList<>(new AtomicInteger(-1), new AtomicInteger(-1));

   private final SideDependentList<AtomicDouble> desiredAbsoluteSteeringAngle = new SideDependentList<>(new AtomicDouble(0.0), new AtomicDouble(0.0));
   private final SideDependentList<AtomicDouble> desiredSteeringSpeedAtomic = new SideDependentList<>(new AtomicDouble(Double.NaN), new AtomicDouble(Double.NaN));

   private final SideDependentList<AtomicBoolean> hasReceivedNewSteeringWheelInformation = new SideDependentList<>(new AtomicBoolean(false), new AtomicBoolean(false));
   private final SideDependentList<AtomicBoolean> hasReceivedNewDesiredSteeringAngle = new SideDependentList<>(new AtomicBoolean(false), new AtomicBoolean(false));

   private final PacketConsumer<DesiredSteeringAnglePacket> desiredSteeringAngleProvider;

   private final GlobalDataProducer globalDataProducer;

   public DesiredSteeringWheelProvider(final GlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      desiredSteeringAngleProvider = new PacketConsumer<DesiredSteeringAnglePacket>()
      {
         @Override
         public void receivedPacket(DesiredSteeringAnglePacket packet)
         {
            if (packet == null)
               return;

            if (!PacketValidityChecker.validateDesiredSteeringAnglePacket(packet, steeringWheelIdAtomic, globalDataProducer))
               return;

            RobotSide robotSide = packet.getRobotSide();

            desiredAbsoluteSteeringAngle.get(robotSide).set(packet.getDesiredAbsoluteSteeringAngle());

            hasReceivedNewDesiredSteeringAngle.get(robotSide).set(true);
         }
      };
   }

   public PacketConsumer<DesiredSteeringAnglePacket> getDesiredSteeringAngleProvider()
   {
      return desiredSteeringAngleProvider;
   }

   @Override
   public void receivedPacket(SteeringWheelInformationPacket packet)
   {
      if (packet == null)
         return;

      if (!PacketValidityChecker.validateSteeringWheelInformationPacket(packet, steeringWheelIdAtomic, globalDataProducer))
         return;

      RobotSide robotSide = packet.getRobotSide();
      steeringWheelCenterAtomic.get(robotSide).set(packet.getSteeringWheelCenter());
      steeringWheelRotationAxisAtomic.get(robotSide).set(packet.getSteeringWheelRotationAxis());
      steeringWheelZeroAxisAtomic.get(robotSide).set(packet.getSteeringWheelZeroAxis());
      steeringWheelRadiusAtomic.get(robotSide).set(packet.getSteeringWheelRadius());
      graspOffsetFromCotnrolFrameAtomic.get(robotSide).set(packet.getGraspOffsetFromControlFrame());
      desiredSteeringSpeedAtomic.get(robotSide).set(packet.getDesiredSteeringSpeed());
      steeringWheelIdAtomic.get(robotSide).set(packet.getSteeringWheelId());

      hasReceivedNewSteeringWheelInformation.get(robotSide).set(true);
   }

   public boolean checkForNewSteeringWheelInformation(RobotSide robotSide)
   {
      return hasReceivedNewSteeringWheelInformation.get(robotSide).getAndSet(false);   
   }

   public boolean checkForNewDesiredAbsoluteSteeringAngle(RobotSide robotSide)
   {
      return hasReceivedNewDesiredSteeringAngle.get(robotSide).getAndSet(false);
   }

   public void getSteeringWheelPose(RobotSide robotSide, FramePoint centerToPack, FrameVector rotationAxisToPack, FrameVector zeroAxisToPack)
   {
      centerToPack.setIncludingFrame(worldFrame, steeringWheelCenterAtomic.get(robotSide).getAndSet(null));
      rotationAxisToPack.setIncludingFrame(worldFrame, steeringWheelRotationAxisAtomic.get(robotSide).getAndSet(null));
      zeroAxisToPack.setIncludingFrame(worldFrame, steeringWheelZeroAxisAtomic.get(robotSide).getAndSet(null));
   }

   public double getSteeringWheelRadius(RobotSide robotSide)
   {
      return steeringWheelRadiusAtomic.get(robotSide).get();
   }

   public double getGraspOffsetFromControlFrame(RobotSide robotSide)
   {
      return graspOffsetFromCotnrolFrameAtomic.get(robotSide).get();
   }

   public double getDesiredAbsoluteSteeringAngle(RobotSide robotSide)
   {
      return desiredAbsoluteSteeringAngle.get(robotSide).get();
   }

   public double getDesiredSteeringSpeed(RobotSide robotSide)
   {
      return desiredSteeringSpeedAtomic.get(robotSide).get();
   }
}

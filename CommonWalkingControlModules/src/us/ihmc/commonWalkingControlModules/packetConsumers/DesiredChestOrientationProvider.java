package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Quat4d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import com.google.common.util.concurrent.AtomicDouble;

public class DesiredChestOrientationProvider implements PacketConsumer<ChestOrientationPacket>, ChestOrientationProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final PacketConsumer<WholeBodyTrajectoryPacket> wholeBodyTrajectoryPacketConsumer;

   private final AtomicReference<Quat4d> desiredOrientation = new AtomicReference<>();
   private final AtomicDouble trajectoryTime = new AtomicDouble();
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final ReferenceFrame chestOrientationFrame;

   private final AtomicReference<WaypointOrientationTrajectoryData> desiredChestOrientationWithWaypoints = new AtomicReference<>(null);

   private final GlobalDataProducer globalDataProducer;

   public DesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame, double defaultTrajectoryTime, GlobalDataProducer globalDataProducer)
   {
      this.chestOrientationFrame = chestOrientationFrame;
      this.globalDataProducer = globalDataProducer;
      trajectoryTime.set(defaultTrajectoryTime);

      wholeBodyTrajectoryPacketConsumer = new PacketConsumer<WholeBodyTrajectoryPacket>()
            {
         @Override
         public void receivedPacket(WholeBodyTrajectoryPacket packet)
         {
            if (packet != null)
            {
               if(packet.chestWorldOrientation != null )
               {

                  WaypointOrientationTrajectoryData data = new WaypointOrientationTrajectoryData(worldFrame, 
                        packet.timeAtWaypoint, 
                        packet.chestWorldOrientation, 
                        packet.chestAngularVelocity);
                  desiredChestOrientationWithWaypoints.set(data);
               }
            }
         }
            };
   }

   public PacketConsumer<WholeBodyTrajectoryPacket> getWholeBodyTrajectoryPacketConsumer()
   {
      return wholeBodyTrajectoryPacketConsumer;
   }

   @Override
   public boolean checkForNewChestOrientation()
   {
      return desiredOrientation.get() != null;
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      return goToHomeOrientation.getAndSet(false);
   }

   @Override
   public boolean checkForNewChestOrientationWithWaypoints()
   {
      return desiredChestOrientationWithWaypoints.get() != null;
   }

   @Override
   public FrameOrientation getDesiredChestOrientation()
   {
      Quat4d orientation = desiredOrientation.getAndSet(null);
      if (orientation == null)
         return null;

      FrameOrientation frameOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), orientation);
      frameOrientation.changeFrame(chestOrientationFrame);
      return frameOrientation;
   }

   @Override
   public WaypointOrientationTrajectoryData getDesiredChestOrientationWithWaypoints()
   {
      return desiredChestOrientationWithWaypoints.getAndSet(null);
   }

   public void receivedPacket(ChestOrientationPacket object)
   {
      if (object == null)
         return;
      
      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateChestOrientationPacket(object);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(ChestOrientationPacket.class, errorMessage);
            return;
         }
      }

      double desiredTrajectoryTime = object.getTrajectoryTime();
      desiredTrajectoryTime = MathTools.clipToMinMax(desiredTrajectoryTime, 0.0001, 30.0);
      trajectoryTime.set(desiredTrajectoryTime);

      // If go to home orientation requested, ignore the other commands.
      if (object.isToHomeOrientation())
      {
         goToHomeOrientation.set(true);
         return;
      }

      desiredOrientation.set(object.getOrientation());
   }

   @Override
   public ReferenceFrame getChestOrientationExpressedInFrame()
   {
      return chestOrientationFrame;
   }

   @Override
   public double getTrajectoryTime()
   {
      return trajectoryTime.get();
   }
}

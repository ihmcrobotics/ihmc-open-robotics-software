package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTools;
import us.ihmc.commonWalkingControlModules.packetConsumers.PacketValidityChecker;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathConsumer implements PacketConsumer<FootstepDataListMessage>
{
   private boolean DEBUG = false;
   private FootstepPathCoordinator footstepPathCoordinator;
   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final HumanoidGlobalDataProducer globalDataProducer;

   public FootstepPathConsumer(SideDependentList<? extends ContactablePlaneBody> bipedFeet, FootstepPathCoordinator footstepPathCoordinator,
                               HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters, HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
      this.footstepPathCoordinator = footstepPathCoordinator;
      this.bipedFeet = bipedFeet;
   }

   public void receivedPacket(FootstepDataListMessage footstepList)
   {
      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateFootstepDataList(footstepList);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(FootstepDataListMessage.class, errorMessage);
            return;
         }
      }
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

      for (int i = 0; i < footstepList.size(); i++)
      {
         FootstepDataMessage footstepData = footstepList.get(i);
         ContactablePlaneBody contactableBody = bipedFeet.get(footstepData.getRobotSide());
         Footstep footstep = FootstepTools.generateFootstepFromFootstepData(footstepData, contactableBody, i);
         footsteps.add(footstep);

         if (DEBUG)
         {
            System.out.println("FootstepPathConsumer received " + footstep);
         }
      }

      footstepPathCoordinator.setSwingTime(footstepList.swingTime);
      footstepPathCoordinator.setTransferTime(footstepList.transferTime);
      footstepPathCoordinator.updatePath(footsteps);
   }

}

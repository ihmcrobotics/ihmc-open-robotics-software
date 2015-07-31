package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTools;
import us.ihmc.commonWalkingControlModules.packetConsumers.PacketValidityChecker;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.trajectories.providers.TrajectoryParameters;
import us.ihmc.utilities.robotSide.SideDependentList;

/**
 * User: Matt
 * Date: 1/18/13
 */
public class FootstepPathConsumer implements PacketConsumer<FootstepDataList>
{
   private boolean DEBUG = false;
   private FootstepPathCoordinator footstepPathCoordinator;
   private final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final GlobalDataProducer globalDataProducer;

   public FootstepPathConsumer(SideDependentList<? extends ContactablePlaneBody> bipedFeet, FootstepPathCoordinator footstepPathCoordinator,
                               HashMap<Footstep, TrajectoryParameters> mapFromFootstepsToTrajectoryParameters, GlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
      this.footstepPathCoordinator = footstepPathCoordinator;
      this.bipedFeet = bipedFeet;
   }

   public void receivedPacket(FootstepDataList footstepList)
   {
      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateFootstepDataList(footstepList);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(FootstepDataList.class, errorMessage);
            return;
         }
      }
      ArrayList<Footstep> footsteps = new ArrayList<Footstep>();

      for (int i = 0; i < footstepList.size(); i++)
      {
         FootstepData footstepData = footstepList.get(i);
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

package us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class StatusBasedBipedalSupportPlanarRegionCalculator
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final OneDoFJointBasics[] allJointsExcludingHands;
   private final HumanoidReferenceFrames referenceFrames;
   private final BipedalSupportPlanarRegionCalculator supportRegionCalculator;

   private final SideDependentList<Boolean> isInContact = new SideDependentList<>();

   public StatusBasedBipedalSupportPlanarRegionCalculator(DRCRobotModel robotModel)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      supportRegionCalculator = new BipedalSupportPlanarRegionCalculator(fullRobotModel, referenceFrames, robotModel.getContactPointParameters(), isInContact::get);
   }

   public void initializeEmptyRegions()
   {
      supportRegionCalculator.initializeEmptyRegions();
   }

   public void calculateSupportRegions(double scaleFactor, CapturabilityBasedStatus capturabilityBasedStatus, RobotConfigurationData robotConfigurationData)
   {
      KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, fullRobotModel.getRootJoint(), allJointsExcludingHands);

      referenceFrames.updateFrames();

      isInContact.put(RobotSide.LEFT, !capturabilityBasedStatus.getLeftFootSupportPolygon3d().isEmpty());
      isInContact.put(RobotSide.RIGHT, !capturabilityBasedStatus.getRightFootSupportPolygon3d().isEmpty());

      supportRegionCalculator.calculateSupportRegions(scaleFactor);
   }

   public List<PlanarRegion> getSupportRegions()
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();
      for (PlanarRegionCommand command : supportRegionCalculator.getSupportRegions())
      {
         if (command == null)
            continue;
         PlanarRegion planarRegion = new PlanarRegion();
         command.getPlanarRegion(planarRegion);
         planarRegions.add(planarRegion);
      }
      return planarRegions;
   }

   public PlanarRegionsList getSupportRegionsAsList()
   {
      return new PlanarRegionsList(getSupportRegions());
   }
}

package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasJointPrivilegedConfigurationParameters;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasSwingTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasKinematicSimulation
{
   public static HumanoidKinematicsSimulation create(AtlasRobotModel robotModel, HumanoidKinematicsSimulationParameters kinematicsSimulationParameters)
   {
      AtlasWalkingControllerParameters walkingControllerParameters = (AtlasWalkingControllerParameters) robotModel.getWalkingControllerParameters();
      walkingControllerParameters.setDoPrepareManipulationForLocomotion(false);
      walkingControllerParameters.setSteppingParameters(new AtlasKinematicSteppingParameters(robotModel.getJointMap()));
      walkingControllerParameters.setSwingTrajectoryParameters(
            new AtlasKinematicSwingTrajectoryParameters(robotModel.getTarget(), robotModel.getJointMap().getModelScale()));
      walkingControllerParameters.setJointPrivilegedConfigurationParameters(
            new AtlasKinematicJointPrivilegedConfigurationParameters(robotModel.getTarget() == RobotTarget.REAL_ROBOT));
      return HumanoidKinematicsSimulation.create(robotModel, kinematicsSimulationParameters);
   }

   static class AtlasKinematicSteppingParameters extends AtlasSteppingParameters
   {
      public AtlasKinematicSteppingParameters(AtlasJointMap jointMap)
      {
         super(jointMap);
      }

      @Override
      public double getMinSwingHeightFromStanceFoot()
      {
         return 0.05 * jointMap.getModelScale();
      }
   }

   static class AtlasKinematicSwingTrajectoryParameters extends AtlasSwingTrajectoryParameters
   {
      public AtlasKinematicSwingTrajectoryParameters(RobotTarget target, double modelScale)
      {
         super(target, modelScale);
      }

      @Override
      public boolean addOrientationMidpointForObstacleClearance()
      {
         return true;
      }
   }

   static class AtlasKinematicJointPrivilegedConfigurationParameters extends AtlasJointPrivilegedConfigurationParameters
   {
      public AtlasKinematicJointPrivilegedConfigurationParameters(boolean runningOnRealRobot)
      {
         super(runningOnRealRobot);
      }

      @Override
      public double getNullspaceProjectionAlpha()
      {
         return 0.1;
      }
   }

   public static void main(String[] args)
   {
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(PubSubImplementation.FAST_RTPS);
      AtlasKinematicSimulation.create(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false),
                                      kinematicsSimulationParameters);
   }
}

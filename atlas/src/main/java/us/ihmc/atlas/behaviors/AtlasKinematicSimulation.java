package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasJointPrivilegedConfigurationParameters;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasSwingTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasKinematicSimulation
{
   public static void create(AtlasRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      AtlasWalkingControllerParameters walkingControllerParameters = (AtlasWalkingControllerParameters) robotModel.getWalkingControllerParameters();
      AtlasSteppingParameters steppingParameters = (AtlasSteppingParameters) walkingControllerParameters.getSteppingParameters();
      steppingParameters.setMinSwingHeightFromStanceFootScalar(0.05);
      AtlasSwingTrajectoryParameters swingTrajectoryParameters = (AtlasSwingTrajectoryParameters) walkingControllerParameters.getSwingTrajectoryParameters();
      swingTrajectoryParameters.setAddOrientationMidpointForObstacleClearance(true);
      AtlasJointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters = (AtlasJointPrivilegedConfigurationParameters) walkingControllerParameters.getJointPrivilegedConfigurationParameters();
      jointPrivilegedConfigurationParameters.setNullspaceProjectionAlpha(0.1);
      HumanoidKinematicsSimulation.create(robotModel, createYoVariableServer, pubSubImplementation);
   }

   public static void main(String[] args)
   {
      AtlasKinematicSimulation.create(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false),
                                      false,
                                      PubSubImplementation.FAST_RTPS);
   }
}

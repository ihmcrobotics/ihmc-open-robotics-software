package us.ihmc.zulu.rdx;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTestFacilitator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.parameters.model.ZuluSimulationCollisionModel;

public class ZuluRDXBehaviorTestFacilitator
      extends RDXBehaviorTestFacilitator
{
   private static final String SVO_FILE = System.getProperty("user.home") + "/Downloads/_.svo2"; // Fill in if you have one

   public ZuluRDXBehaviorTestFacilitator()
   {
      super(SVO_FILE,
            "zulu",
            () -> new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS),
            (robotModel, ros2Node, initialWalkingPose) ->
            {
               HumanoidKinematicsSimulationParameters parameters = new HumanoidKinematicsSimulationParameters();
               parameters.setROS2Node(ros2Node);
               return HumanoidKinematicsSimulation.create(robotModel, parameters);
            },
            null,
            () -> new RDXBaseUI(ZuluRDXBehaviorTestFacilitator.class),
            new WorkspaceResourceDirectory(ZuluRDXBehaviorTestFacilitator.class, "/behaviorTrees"),
            robotModel -> new ZuluSimulationCollisionModel(robotModel.getJointMap()));
   }

   public static void main(String[] args)
   {
      new ZuluRDXBehaviorTestFacilitator();
   }
}

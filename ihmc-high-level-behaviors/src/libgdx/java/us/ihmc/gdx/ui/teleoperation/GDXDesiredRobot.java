package us.ihmc.gdx.ui.teleoperation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.gdx.ui.graphics.GDXMultiBodyGraphic;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

/**
 *  This is the class that is updated based on the desired values of the robot. If poses get sent to the robot, it should be these poses.
 *
 *  TODO make this robot color differently.
  */
public class GDXDesiredRobot extends GDXMultiBodyGraphic
{
   private final ROS2SyncedRobotModel syncedRobotModel;
   private final DRCRobotModel robotModel;
   private final FullHumanoidRobotModel desiredFullRobotModel;

   public GDXDesiredRobot(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobotModel)
   {
      super(robotModel.getSimpleRobotName() + " Desired Robot Visualizer");

      this.robotModel = robotModel;
      this.syncedRobotModel = syncedRobotModel;
      desiredFullRobotModel = robotModel.createFullRobotModel();

      super.setActive(true);
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   @Override
   public void create()
   {
      super.create();

      RobotDefinition robotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      ColorDefinition ghostColor = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
      MaterialDefinition material = new MaterialDefinition(ghostColor);
      RobotDefinition.forEachRigidBodyDefinition(robotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
      loadRobotModelAndGraphics(robotDefinition, desiredFullRobotModel.getElevator());
   }

   @Override
   public void update()
   {
      if (isRobotLoaded())
      {
         super.update();
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
   }

   public void destroy()
   {
      super.destroy();
   }

   public void setDesiredToCurrent()
   {
      desiredFullRobotModel.getRootJoint().setJointConfiguration(syncedRobotModel.getFullRobotModel().getRootJoint());
      for (OneDoFJointBasics joint : syncedRobotModel.getFullRobotModel().getOneDoFJoints())
         desiredFullRobotModel.getOneDoFJointByName(joint.getName()).setJointConfiguration(joint);
   }
}

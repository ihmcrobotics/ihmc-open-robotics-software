package us.ihmc.gdx.ui.teleoperation;

import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.gdx.ui.graphics.GDXMultiBodyGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.InverseDynamicsJointStateCopier;
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
      super(robotModel.getSimpleRobotName() + "Desired Robot Visualizer");

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

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      ColorDefinition ghostColor = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
      MaterialDefinition material = new MaterialDefinition(ghostColor);
      for (RobotSide robotSide : RobotSide.values)
      {
         String handName = robotModel.getJointMap().getHandName(robotSide);
         RobotDefinition.forEachRigidBodyDefinition(robotDefinition.getRigidBodyDefinition(handName),
                                                    body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
      }
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

package us.ihmc.gdx.ui.teleoperation;

import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.gdx.ui.graphics.GDXMultiBodyGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

/**
 *  This is the class that is updated based on the desired values of the robot. If poses get sent to the robot, it should be these poses.
 *
 *  TODO make this robot color differently.
  */
public class GDXDesiredRobot extends GDXMultiBodyGraphic
{
   private final DRCRobotModel robotModel;
   private final FullHumanoidRobotModel desiredFullRobotModel;

   public GDXDesiredRobot(DRCRobotModel robotModel)
   {
      super(robotModel.getSimpleRobotName() + "Desired Robot Visualizer");

      this.robotModel = robotModel;
      desiredFullRobotModel = robotModel.createFullRobotModel();
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
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.Black());
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

}

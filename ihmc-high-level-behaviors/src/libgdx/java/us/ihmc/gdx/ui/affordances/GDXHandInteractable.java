package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXSpatialVectorArrows;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class GDXHandInteractable extends GDXLiveRobotPartInteractable
{
   private final RobotSide side;
   private final ROS2SyncedRobotModel syncedRobot;
   private GDXSpatialVectorArrows wristWrenchArrows;
   private String contextMenuName;

   public static boolean collisionLinkIsHand(RobotSide side, GDXRobotCollisionLink collisionLink, FullHumanoidRobotModel fullRobotModel)
   {
      return collisionLink.getRigidBodyName().equals(fullRobotModel.getHand(side).getName());
   }

   public GDXHandInteractable(RobotSide side,
                              GDXImGuiBasedUI baseUI,
                              GDXRobotCollisionLink collisionLink,
                              DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              YoVariableClientHelper yoVariableClientHelper)
   {
      this.side = side;
      this.syncedRobot = syncedRobot;

      String robotSidePrefix = (side == RobotSide.LEFT) ? "l_" : "r_";
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      FullHumanoidRobotModel fullRobotModel = syncedRobot.getFullRobotModel();
      String modelFileName = GDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(collisionLink.getRigidBodyName()));

      ReferenceFrame handFrame = fullRobotModel.getEndEffectorFrame(side, LimbName.ARM);
      ReferenceFrame collisionFrame = handFrame;
      ReferenceFrame handControlFrame = fullRobotModel.getHandControlFrame(side);
      ReferenceFrame handGraphicFrame
            = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(robotSidePrefix + "graphicFrame",
                                                                                handFrame,
                                                                                robotModel.getUIParameters().getHandGraphicToHandFrameTransform(side));
      super.create(collisionLink, handGraphicFrame, collisionFrame, handControlFrame, modelFileName, baseUI.getPrimary3DPanel());

      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         if (wristForceSensorNames.containsKey(side) && wristForceSensorNames.get(side).equals(forceSensorDefinitions[i].getSensorName()))
         {
            // wristWrenchArrows.put(side, new GDXSpatialVectorArrows(forceSensorDefinitions[i].getSensorFrame(), i));
            wristWrenchArrows = new GDXSpatialVectorArrows(forceSensorDefinitions[i].getSensorFrame(),
                                                           yoVariableClientHelper,
                                                           side.getLowerCaseName() + "WristSensor");
         }
      }

      contextMenuName = side + " Hand Context Menu";
   }

   public void update()
   {
      if (wristWrenchArrows != null)
      {
         // GDXSpatialVectorArrows wristArrows = wristWrenchArrows.get(side);
         // if (syncedRobot.getForceSensorData().size() > wristArrows.getIndexOfSensor())
         // {
         //    SpatialVectorMessage forceSensorData = syncedRobot.getForceSensorData().get(wristArrows.getIndexOfSensor());
         //    wristArrows.update(forceSensorData.getLinearPart(), forceSensorData.getAngularPart());
         // }
         wristWrenchArrows.updateFromYoVariables();
      }
   }

   @Override
   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getVirtualRenderables(renderables, pool);

      if (wristWrenchArrows != null)
      {
         if (syncedRobot.getForceSensorData().size() > wristWrenchArrows.getIndexOfSensor())
         {
            wristWrenchArrows.getRenderables(renderables, pool);
         }
      }
   }

   public String getContextMenuName()
   {
      return contextMenuName;
   }
}

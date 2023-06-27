package us.ihmc.rdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXSpatialVectorArrows;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.wholeBodyController.HandTransformTools;

/**
 * This class manages being able to click on a humanoid robot's hand in the UI
 * and move a preview/desired version of it around. There's some complicated
 * frame operations involved in this.
 * <br/>
 * See {@link RDXInteractableRobotLink} for more explanation.
 */
public class RDXInteractableHand extends RDXInteractableRobotLink
{
   private final RobotSide side;
   private final ROS2SyncedRobotModel syncedRobot;
   private RDXSpatialVectorArrows sensorWristWrenchArrows;
   private final RDXSpatialVectorArrows estimatedHandWrenchArrows;
   private final String contextMenuName;

   public static boolean robotCollidableIsHand(RobotSide side, RDXRobotCollidable robotCollidable, FullHumanoidRobotModel fullRobotModel)
   {
      return robotCollidable.getRigidBodyName().equals(fullRobotModel.getHand(side).getName());
   }

   public RDXInteractableHand(RobotSide side,
                              RDXBaseUI baseUI,
                              RDXRobotCollidable robotCollidable,
                              DRCRobotModel robotModel,
                              ROS2SyncedRobotModel syncedRobot,
                              YoVariableClientHelper yoVariableClientHelper)
   {
      this.side = side;
      this.syncedRobot = syncedRobot;

      RobotDefinition robotDefinition = robotModel.getRobotDefinition();
      FullHumanoidRobotModel fullRobotModel = syncedRobot.getFullRobotModel();
      String modelFileName = RDXInteractableTools.getModelFileName(robotDefinition.getRigidBodyDefinition(robotCollidable.getRigidBodyName()));

      ReferenceFrame syncedControlFrame = fullRobotModel.getHandControlFrame(side);
      RigidBodyTransform graphicToControlFrameTransform = new RigidBodyTransform();
      RigidBodyTransform linkToControlFrameTransform = new RigidBodyTransform();
      HandTransformTools.getHandLinkToControlFrameTransform(fullRobotModel, side, linkToControlFrameTransform);
      HandTransformTools.getHandGraphicToControlFrameTransform(fullRobotModel, robotModel.getUIParameters(), side, graphicToControlFrameTransform);
      super.create(robotCollidable, syncedControlFrame, graphicToControlFrameTransform, linkToControlFrameTransform, modelFileName, baseUI.getPrimary3DPanel());

      HumanoidRobotSensorInformation sensorInformation = robotModel.getSensorInformation();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      for (int i = 0; i < forceSensorDefinitions.length; i++)
      {
         if (wristForceSensorNames.containsKey(side) && wristForceSensorNames.get(side).equals(forceSensorDefinitions[i].getSensorName()))
         {
            // wristWrenchArrows.put(side, new RDXSpatialVectorArrows(forceSensorDefinitions[i].getSensorFrame(), i));
            sensorWristWrenchArrows = new RDXSpatialVectorArrows(forceSensorDefinitions[i].getSensorFrame(),
                                                                 yoVariableClientHelper,
                                                                 side.getLowerCaseName() + "WristSensor");
         }
      }
      ReferenceFrame afterLastWristJointFrame = fullRobotModel.getEndEffectorFrame(side, LimbName.ARM);
      estimatedHandWrenchArrows = new RDXSpatialVectorArrows(afterLastWristJointFrame);
      estimatedHandWrenchArrows.setAngularPartScale(0.05);
      contextMenuName = side + " Hand Context Menu";
   }

   @Override
   public void update()
   {
      super.update();

      if (sensorWristWrenchArrows != null)
      {
         // RDXSpatialVectorArrows wristArrows = wristWrenchArrows.get(side);
         // if (syncedRobot.getForceSensorData().size() > wristArrows.getIndexOfSensor())
         // {
         //    SpatialVectorMessage forceSensorData = syncedRobot.getForceSensorData().get(wristArrows.getIndexOfSensor());
         //    wristArrows.update(forceSensorData.getLinearPart(), forceSensorData.getAngularPart());
         // }
         sensorWristWrenchArrows.updateFromYoVariables();
      }
   }

   public void updateEstimatedWrench(SpatialVectorReadOnly spatialVector)
   {
      estimatedHandWrenchArrows.update(spatialVector);
   }

   @Override
   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getVirtualRenderables(renderables, pool);

      if (sensorWristWrenchArrows != null)
      {
         if (syncedRobot.getForceSensorData().size() > sensorWristWrenchArrows.getIndexOfSensor())
         {
            sensorWristWrenchArrows.getRenderables(renderables, pool);
         }
      }
      estimatedHandWrenchArrows.getRenderables(renderables, pool);
   }

   public String getContextMenuName()
   {
      return contextMenuName;
   }

   public RDXSpatialVectorArrows getEstimatedHandWrenchArrows()
   {
      return estimatedHandWrenchArrows;
   }
}

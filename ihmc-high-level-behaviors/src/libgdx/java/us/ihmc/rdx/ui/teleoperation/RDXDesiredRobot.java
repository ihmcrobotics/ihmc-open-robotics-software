package us.ihmc.rdx.ui.teleoperation;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.rdx.simulation.scs2.RDXVisualTools;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2DefinitionMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;

import java.util.ArrayList;
import java.util.List;

/**
 *  This is the class that is updated based on the desired values of the robot. If poses get sent to the robot, it should be these poses.
 *
 *  TODO make this robot color differently.
 */
public class RDXDesiredRobot extends RDXMultiBodyGraphic
{
   private final DRCRobotModel robotModel;
   private RobotDefinition robotDefinition;
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final String pelvisRigidBodyName;
   private final String chestRigidBodyName;
   private final SideDependentList<ArrayList<String>> armRigidBodyNames = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private final SideDependentList<ArrayList<String>> legRigidBodyNames = new SideDependentList<>(new ArrayList<>(), new ArrayList<>());
   private boolean pelvisShowing = false;
   private boolean chestShowing = false;
   private final SideDependentList<Boolean> armShowing = new SideDependentList<>(false, false);
   private final SideDependentList<Boolean> legShowing = new SideDependentList<>(false, false);

   public RDXDesiredRobot(DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobotModel)
   {
      super(robotModel.getSimpleRobotName() + " Desired Robot Visualizer");

      this.robotModel = robotModel;
      desiredFullRobotModel = robotModel.createFullRobotModel();

      pelvisRigidBodyName = robotModel.getJointMap().getPelvisName();
      chestRigidBodyName = robotModel.getJointMap().getChestName();

      super.setActive(true);
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   public DRCRobotModel getDesiredRobotModel()
   {
      return robotModel;
   }

   @Override
   public void create()
   {
      super.create();

      robotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      RobotDefinition robotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      ColorDefinition ghostColor = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
      MaterialDefinition material = new MaterialDefinition(ghostColor);
      SCS2DefinitionMissingTools.forEachRigidBodyDefinitionIncludingFourBars(robotDefinition.getRootBodyDefinition(),
                                                                             body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
      boolean createReferenceFrameGraphics = false;
      loadRobotModelAndGraphics(robotDefinition, desiredFullRobotModel.getElevator(), RDXVisualTools.DESIRED_ROBOT_SCALING, createReferenceFrameGraphics);
   }

   @Override
   public void update()
   {
      if (isRobotLoaded())
      {
         if (getRobotLoadedActivator().isNewlyActivated())
         {
            for (RobotSide side : RobotSide.values)
            {
               List<String> armJointNamesAsStrings = robotModel.getJointMap().getArmJointNamesAsStrings(side);
               for (String armJointNamesAsString : armJointNamesAsStrings)
               {
                  JointBasics armJoint = MultiBodySystemTools.findJoint(getMultiBody(), armJointNamesAsString);
                  armRigidBodyNames.get(side).add(armJoint.getSuccessor().getName());
               }
               List<String> legJointNamesAsStrings = robotModel.getJointMap().getLegJointNamesAsStrings(side);
               for (String legJointNamesAsString : legJointNamesAsStrings)
               {
                  JointBasics legJoint = MultiBodySystemTools.findJoint(getMultiBody(), legJointNamesAsString);
                  legRigidBodyNames.get(side).add(legJoint.getSuccessor().getName());
               }
            }

            // hide all to start with; probably includes stuff we'll never show, too
            RobotDefinition.forEachRigidBodyDefinition(robotDefinition.getRootBodyDefinition(),
                                                       rigidBody -> getMultiBody().getRigidBodiesToHide().add(rigidBody.getName()));
         }

         // TODO: Scale the ghost robot bigger than actual
         //  getMultiBody().scale(1.01f,1.01f,1.01f);
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

   @Override
   public void setActive(boolean active)
   {
      super.setActive(active);
      if (!active)
      {
         setPelvisShowing(false);
         setChestShowing(false);
         for (RobotSide side : RobotSide.values)
         {
            setArmShowing(side, false);
            setLegShowing(side, false);
         }
      }
   }

   public void setPelvisShowing(boolean showing)
   {
      if (showing != pelvisShowing)
      {
         if (showing)
            getMultiBody().getRigidBodiesToHide().remove(pelvisRigidBodyName);
         else
            getMultiBody().getRigidBodiesToHide().add(pelvisRigidBodyName);

         pelvisShowing = showing;
      }
   }

   public void setChestShowing(boolean showing)
   {
      if (showing != chestShowing)
      {
         if (showing)
            getMultiBody().getRigidBodiesToHide().remove(chestRigidBodyName);
         else
            getMultiBody().getRigidBodiesToHide().add(chestRigidBodyName);

         chestShowing = showing;
      }
   }

   public void setArmShowing(RobotSide side, boolean showing)
   {
      if (showing != armShowing.get(side))
      {
         if (showing)
            for (String armRigidBodyName : armRigidBodyNames.get(side))
               getMultiBody().getRigidBodiesToHide().remove(armRigidBodyName);
         else
            for (String armRigidBodyName : armRigidBodyNames.get(side))
               getMultiBody().getRigidBodiesToHide().add(armRigidBodyName);

         armShowing.put(side, showing);
      }
   }

   public void setLegShowing(RobotSide side, boolean showing)
   {
      if (showing != legShowing.get(side))
      {
         if (showing)
            for (String legRigidBodyName : legRigidBodyNames.get(side))
               getMultiBody().getRigidBodiesToHide().remove(legRigidBodyName);
         else
            for (String legRigidBodyName : legRigidBodyNames.get(side))
               getMultiBody().getRigidBodiesToHide().add(legRigidBodyName);

         legShowing.put(side, showing);
      }
   }
}

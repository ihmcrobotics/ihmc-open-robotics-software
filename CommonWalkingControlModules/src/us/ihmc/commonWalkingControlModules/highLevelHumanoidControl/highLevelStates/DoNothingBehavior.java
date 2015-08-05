package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class DoNothingBehavior extends HighLevelBehavior
{
   private static final HighLevelState controllerState = HighLevelState.DO_NOTHING_BEHAVIOR;

   private final MomentumBasedController momentumBasedController;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();

   private final ArrayList<OneDoFJoint> allRobotJoints = new ArrayList<OneDoFJoint>();

   public DoNothingBehavior(MomentumBasedController momentumBasedController, BipedSupportPolygons bipedSupportPolygons)
   {
      super(controllerState);

      this.bipedSupportPolygons = bipedSupportPolygons;
      this.momentumBasedController = momentumBasedController;
      momentumBasedController.getFullRobotModel().getOneDoFJoints(allRobotJoints);

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
         footContactStates.put(robotSide, momentumBasedController.getContactState(contactableFoot));
      }
   }

   @Override
   public void doAction()
   {
      bipedSupportPolygons.updateUsingContactStates(footContactStates);
      momentumBasedController.callUpdatables();

      for (int i = 0; i < allRobotJoints.size(); i++)
      {
         allRobotJoints.get(i).setTau(0.0);
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      // Do nothing

   }

   @Override
   public void doTransitionOutOfAction()
   {
      // Do nothing

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }
}

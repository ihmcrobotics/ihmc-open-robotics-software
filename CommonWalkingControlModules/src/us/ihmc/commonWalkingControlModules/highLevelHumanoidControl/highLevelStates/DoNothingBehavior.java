package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;

public class DoNothingBehavior extends HighLevelBehavior
{
   private static final HighLevelState controllerState = HighLevelState.DO_NOTHING_BEHAVIOR;

   private final MomentumBasedController momentumBasedController;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<PlaneContactState> footContactStates = new SideDependentList<>();

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

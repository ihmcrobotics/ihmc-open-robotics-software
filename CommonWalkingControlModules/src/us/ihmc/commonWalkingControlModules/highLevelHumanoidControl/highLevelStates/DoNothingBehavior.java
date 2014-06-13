package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class DoNothingBehavior extends HighLevelBehavior
{
   private static final HighLevelState controllerState = HighLevelState.DO_NOTHING_BEHAVIOR;

   private final MomentumBasedController momentumBasedController;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<PlaneContactState> footContactStates = new SideDependentList<>();
   
   public DoNothingBehavior(MomentumBasedController momentumBasedController, BipedSupportPolygons bipedSupportPolygons)
   {
      super(controllerState);
      
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.momentumBasedController = momentumBasedController;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = momentumBasedController.getContactablePlaneFeet().get(robotSide);
         footContactStates.put(robotSide, momentumBasedController.getContactState(contactableFoot));
      }
   }

   @Override
   public void doAction()
   {
      bipedSupportPolygons.updateUsingContactStates(footContactStates);
      momentumBasedController.callUpdatables();
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

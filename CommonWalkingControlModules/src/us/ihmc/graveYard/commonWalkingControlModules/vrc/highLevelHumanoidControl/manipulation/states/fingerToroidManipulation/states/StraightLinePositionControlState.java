package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlModule;
import us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.fingerToroidManipulation.FingerToroidManipulationState;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.SE3ConfigurationProvider;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.SE3PIDGains;
import us.ihmc.yoUtilities.stateMachines.State;


/**
 * @author twan
 *         Date: 5/21/13
 */
public class StraightLinePositionControlState extends State<FingerToroidManipulationState>
{
   private final SideDependentList<HandControlModule> individualHandControlModules;
   private final SideDependentList<SE3ConfigurationProvider> finalConfigurationProviders;
   private final SideDependentList<ReferenceFrame> handPositionControlFrames;

   private final FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final FramePoint position = new FramePoint(ReferenceFrame.getWorldFrame());
   private final double trajectoryTime;
   private final RigidBody base;
   private final ReferenceFrame trajectoryFrame = ReferenceFrame.getWorldFrame();
   private final FramePose finalDesiredPose = new FramePose(trajectoryFrame);
   private final SE3PIDGains gains;

   public StraightLinePositionControlState(FingerToroidManipulationState stateEnum,
                                           SideDependentList<HandControlModule> individualHandControlModules, RigidBody rootBody,
                                           SideDependentList<SE3ConfigurationProvider> finalConfigurationProviders, SideDependentList<ReferenceFrame> handPositionControlFrames,
                                           double trajectoryTime, SE3PIDGains gains)
   {
      super(stateEnum);
      this.individualHandControlModules = individualHandControlModules;
      this.finalConfigurationProviders = finalConfigurationProviders;
      this.trajectoryTime = trajectoryTime;
      this.base = rootBody;
      this.handPositionControlFrames = handPositionControlFrames;
      this.gains = gains;
   }

   @Override
   public void doAction()
   {
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         finalConfigurationProviders.get(robotSide).get(orientation);
         orientation.changeFrame(trajectoryFrame);

         finalConfigurationProviders.get(robotSide).get(position);
         position.changeFrame(trajectoryFrame);

         finalDesiredPose.setToZero(orientation.getReferenceFrame());
         finalDesiredPose.setOrientation(orientation);
         finalDesiredPose.setPosition(position);
         finalDesiredPose.changeFrame(trajectoryFrame);

         ReferenceFrame handPositionControlFrame = handPositionControlFrames.get(robotSide);
//         individualHandControlModules.get(robotSide).moveInStraightLine(finalDesiredPose, trajectoryTime, base, handPositionControlFrame,
//               trajectoryFrame, gains);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO: automatically generated code
   }

   @Override
   public boolean isDone()
   {
      for (HandControlModule individualHandControlModule : individualHandControlModules)
      {
         if (!individualHandControlModule.isDone())
            return false;
      }
      return true;
   }
}

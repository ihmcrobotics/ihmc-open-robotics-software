package us.ihmc.simpleWholeBodyWalking;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simpleWholeBodyWalking.SimpleFootControlModule.ConstraintType;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimpleFeetManager
{
   private static final boolean USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<SimpleFootControlModule> footControlModules = new SideDependentList<>();

   private final SideDependentList<ContactableFoot> feet;

   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final SideDependentList<FootSwitchInterface> footSwitches;

   private final FramePoint3D tempSolePosition = new FramePoint3D();
   private final DoubleParameter blindFootstepsHeightOffset;

   public SimpleFeetManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                            WalkingControllerParameters walkingControllerParameters,
                            PIDSE3GainsReadOnly swingFootGains,
                            PIDSE3GainsReadOnly holdFootGains,
                            YoRegistry parentRegistry)
   {
      feet = controllerToolbox.getContactableFeet();

      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
         contactStates.put(robotSide, controllerToolbox.getFootContactState(robotSide));

      this.footSwitches = controllerToolbox.getFootSwitches();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      soleZUpFrames = referenceFrames.getSoleZUpFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         SimpleFootControlModule footControlModule = new SimpleFootControlModule(robotSide,
                                                                                 walkingControllerParameters,
                                                                                 swingFootGains,
                                                                                 holdFootGains,
                                                                                 controllerToolbox,
                                                                                 registry);

         footControlModules.put(robotSide, footControlModule);
      }

      double defaultBlindFootstepsHeightOffset = walkingControllerParameters.getSwingTrajectoryParameters().getBlindFootstepsHeightOffset();
      blindFootstepsHeightOffset = new DoubleParameter("blindFootstepsHeightOffset", registry, defaultBlindFootstepsHeightOffset);

      parentRegistry.addChild(registry);
   }

   public void setWeights(Vector3DReadOnly loadedFootAngularWeight,
                          Vector3DReadOnly loadedFootLinearWeight,
                          Vector3DReadOnly footAngularWeight,
                          Vector3DReadOnly footLinearWeight)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         SimpleFootControlModule footControlModule = footControlModules.get(robotSide);
         footControlModule.setWeights(loadedFootAngularWeight, loadedFootLinearWeight, footAngularWeight, footLinearWeight);
      }
   }

   public void initialize()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footControlModules.get(robotSide).initialize();
      }
   }

   public void compute()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footSwitches.get(robotSide).hasFootHitGround(); //debug
         footControlModules.get(robotSide).doControl();
      }
   }

   public boolean adjustHeightIfNeeded(Footstep footstep)
   {
      if (!footstep.getTrustHeight())
      {
         tempSolePosition.setToZero(soleZUpFrames.get(footstep.getRobotSide().getOppositeSide()));
         tempSolePosition.changeFrame(footstep.getFootstepPose().getReferenceFrame());
         footstep.setZ(tempSolePosition.getZ() + blindFootstepsHeightOffset.getValue());
         return true;
      }
      return false;
   }

   public void requestSwing(RobotSide upcomingSwingSide, Footstep footstep, double swingTime)
   {
      SimpleFootControlModule footControlModule = footControlModules.get(upcomingSwingSide);
      footControlModule.setFootstep(footstep, swingTime);
      setContactStateForSwing(upcomingSwingSide);
   }

   public ConstraintType getCurrentConstraintType(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getCurrentConstraintType();
   }

   public void adjustSwingTrajectory(RobotSide swingSide, Footstep adjustedFootstep, double swingTime)
   {
      footControlModules.get(swingSide).setAdjustedFootstepAndTime(adjustedFootstep, swingTime);
   }

   public void initializeContactStatesForDoubleSupport(RobotSide transferToSide)
   {
      if (transferToSide == null)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            setFlatFootContactState(robotSide);
         }
      }
      else
      {
         if (getCurrentConstraintType(transferToSide.getOppositeSide()) == ConstraintType.SWING) // That case happens when doing 2 steps on same side
            setFlatFootContactState(transferToSide.getOppositeSide());
         setFlatFootContactState(transferToSide); // still need to determine contact state for trailing leg. This is done in doAction as soon as the previous ICP trajectory is done
      }
   }

   private final FrameVector3D footNormalContactVector = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);

   public void setFlatFootContactState(RobotSide robotSide)
   {
      if (USE_WORLDFRAME_SURFACE_NORMAL_WHEN_FULLY_CONSTRAINED)
         footNormalContactVector.setIncludingFrame(worldFrame, 0.0, 0.0, 1.0);
      else
         footNormalContactVector.setIncludingFrame(feet.get(robotSide).getSoleFrame(), 0.0, 0.0, 1.0);
      footControlModules.get(robotSide).setContactState(ConstraintType.FULL, footNormalContactVector);
   }

   public void setContactStateForSwing(RobotSide robotSide)
   {
      SimpleFootControlModule footControlModule = footControlModules.get(robotSide);
      footControlModule.setContactState(ConstraintType.SWING);
   }


   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor multiplier on the current time
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(RobotSide robotSide, double speedUpFactor)
   {
      return footControlModules.get(robotSide).requestSwingSpeedUp(speedUpFactor);
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getInverseDynamicsCommand();
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).getFeedbackControlCommand();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         FeedbackControlCommandList template = footControlModules.get(robotSide).createFeedbackControlTemplate();
         for (int i = 0; i < template.getNumberOfCommands(); i++)
            ret.addCommand(template.getCommand(i));
      }
      return ret;
   }

   public Object pollStatusToReport(RobotSide robotSide)
   {
      return footControlModules.get(robotSide).pollStatusToReport();
   }
}

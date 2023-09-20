package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external.ExternalControlCommandConsumer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external.WholeBodyConfigurationManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class ExternalControllerState extends HighLevelControllerState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final ExternalControlCommandConsumer externalControlCommandConsumer;
   private final WholeBodyConfigurationManager wholeBodyConfigurationManager;

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   // Fields for the status output manager
   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();
   private final FramePoint3D centerOfMassPosition = new FramePoint3D();

   public ExternalControllerState(CommandInputManager commandInputManager,
                                  StatusMessageOutputManager statusOutputManager,
                                  HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      super("externalController", HighLevelControllerName.EXTERNAL, MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class));

      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.controllerToolbox = controllerToolbox;

      wholeBodyConfigurationManager = new WholeBodyConfigurationManager(controllerToolbox.getYoTime(),
                                                                        controllerToolbox.getFullRobotModel(),
                                                                        controlledJoints,
                                                                        registry);
      externalControlCommandConsumer = new ExternalControlCommandConsumer(commandInputManager, wholeBodyConfigurationManager, controllerToolbox.getYoTime());
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return wholeBodyConfigurationManager.getControlOutput();
   }

   @Override
   public void onEntry()
   {
      commandInputManager.clearAllCommands();

      wholeBodyConfigurationManager.initialize();
   }

   @Override
   public void doAction(double timeInState)
   {
      externalControlCommandConsumer.consumeExternalControlCommands();

      wholeBodyConfigurationManager.doControl();

      statusOutputManager.reportStatusMessage(updateAndReturnCapturabilityBasedStatus());
   }


   @Override
   public void onExit(double timeInState)
   {
   }

   public CapturabilityBasedStatus updateAndReturnCapturabilityBasedStatus()
   {
      centerOfMassPosition.setToZero(controllerToolbox.getCenterOfMassFrame());
      centerOfMassPosition.changeFrame(worldFrame);

      capturabilityBasedStatus.setOmega(controllerToolbox.getOmega0());
      capturabilityBasedStatus.getCapturePoint2d().set(controllerToolbox.getCapturePoint());
      // TODO we should probably synthesize this from something else.
      capturabilityBasedStatus.getDesiredCapturePoint2d().set(controllerToolbox.getCapturePoint());
      capturabilityBasedStatus.getCenterOfMass3d().set(centerOfMassPosition);
      for (RobotSide robotSide : RobotSide.values)
      {
         HumanoidMessageTools.packFootSupportPolygon(robotSide,
                                                     controllerToolbox.getBipedSupportPolygons().getFootPolygonInSoleFrame(robotSide),
                                                     capturabilityBasedStatus);
      }

      return capturabilityBasedStatus;
   }
}
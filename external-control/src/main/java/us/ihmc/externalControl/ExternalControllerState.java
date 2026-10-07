package us.ihmc.externalControl;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointControlBlender;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.PairList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.externalControl.library.ExternalControlNativeLibrary;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.HashMap;

public class ExternalControllerState extends HighLevelControllerState
{
   public static boolean REDUCE_YOVARIABLES = false;

   private enum DesiredMode
   {HOLD_POSITION, REXXXXXXX}

   private enum DesiredBehavior
   {STAND, WALK_IN_PLACE, WALK_IN_PLACE_10, WALK_FORWARD, WALK_FORWARD_10}

   private final YoDouble blendRatioCurrentValue;

   private final PairList<OneDoFJointBasics, JointControlBlender> jointCommandBlenders = new PairList<>();
   private final JointDesiredOutputListReadOnly highLevelControllerOutput;

   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final LowLevelOneDoFJointDesiredDataHolder frozenJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder();
   private final LowLevelOneDoFJointDesiredDataHolder externalJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder();

   private final YoEnum<DesiredMode> desiredMode = new YoEnum<>("DesiredExternalMode", registry, DesiredMode.class);
   private final YoEnum<DesiredBehavior> desiredBehavior = new YoEnum<>("DesiredExternalBehavior", registry, DesiredBehavior.class);

   private final SideDependentList<YoFramePoint3D> measuredFootCoPs = new SideDependentList<>();
   private final SideDependentList<YoFrameVector3D> measuredFootForces = new SideDependentList<>();

   private final YoFramePoint3D capturePoint = new YoFramePoint3D("CapturePoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D centerOfMass = new YoFramePoint3D("CenterOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D leftFootPosition = new YoFramePoint3D("leftFootPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D rightFootPosition = new YoFramePoint3D("rightFootPosition", ReferenceFrame.getWorldFrame(), registry);

   private final CommandInputManager commandInputManager;

   private final PairList<OneDoFJointBasics, YoDouble> frozenJointDesireds = new PairList<>();

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final ExternalControl externalControl;

   private final HashMap<String, YoDouble> debugData = new HashMap<>();
   private final ExecutionTimer totalControllerTime = new ExecutionTimer("JavaSideControllerTotalTime", 1.0, registry);

   private final YoEnum<RobotSide> lowestFootSide = new YoEnum<>("lowestFootSide", registry, RobotSide.class, true);
   private final YoDouble minimumHeightDifferenceForSwitchingSide = new YoDouble("minimumHeightDifferenceForSwitchingSide", registry);
   private final YoInteger switchWindowSize = new YoInteger("switchWindowSize", registry);
   private final YoBoolean shouldSwitchSupportSide = new YoBoolean("shouldSwitchSupportSide", registry);
   private final GlitchFilteredYoBoolean filteredShouldSwitchSupportSide = new GlitchFilteredYoBoolean("shouldSwitchSupportSideFiltered",
                                                                                                       registry,
                                                                                                       shouldSwitchSupportSide,
                                                                                                       switchWindowSize);

   private final YoDouble rootHeightOffset = new YoDouble("rootHeightOffset", registry);
   private final YoDouble rootHeightBias = new YoDouble("rootHeightBias", registry);
   private final YoDouble rootHeightOffsetBreakFrequency = new YoDouble("rootHeightOffsetBreakFrequency", registry);
   private final AlphaFilteredYoVariable filteredRootHeightOffset;

   private final YoBoolean externalControlSocketIsOn = new YoBoolean("externalControlSocketIsOn", registry);

   public ExternalControllerState(HighLevelControllerParameters highLevelControllerParameters,
                                  HighLevelHumanoidControllerToolbox controllerToolbox,
                                  OneDoFJointBasics[] controlledJoints,
                                  JointDesiredOutputListReadOnly highLevelControllerOutput,
                                  CommandInputManager commandInputManager)
   {
      super("externalControl", HighLevelControllerName.EXTERNAL, highLevelControllerParameters, controlledJoints);

      this.highLevelControllerOutput = highLevelControllerOutput;
      this.controllerToolbox = controllerToolbox;

      this.commandInputManager = commandInputManager;

      blendRatioCurrentValue = new YoDouble("ExternalControlBlendRatioCurrentValue", registry);

      minimumHeightDifferenceForSwitchingSide.set(0.005);
      switchWindowSize.set(4);

      rootHeightBias.set(-0.05);
      rootHeightOffsetBreakFrequency.set(10.0);
      DoubleProvider alphaProvider = () -> AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(rootHeightOffsetBreakFrequency.getDoubleValue(),
                                                                                                    controllerToolbox.getControlDT());
      filteredRootHeightOffset = new AlphaFilteredYoVariable("filteredRootHeightOffset", registry, alphaProvider);

      ExternalControlNativeLibrary.load();

      desiredMode.set(DesiredMode.HOLD_POSITION);
      desiredBehavior.set(DesiredBehavior.STAND);
      externalControl = new ExternalControl(controllerToolbox.getFullRobotModel().getRootBody(),
                                            highLevelControllerParameters.getStandPrepParameters(),
                                            controlledJoints,
                                            1000.0,
                                            5.0,
                                            registry,
                                            controllerToolbox);

      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
      frozenJointDataHolder.registerJointsWithEmptyData(controlledJoints);
      externalJointDataHolder.registerJointsWithEmptyData(controlledJoints);

      externalControlSocketIsOn.set(true);
      externalControlSocketIsOn.addListener(v ->
      {
         if (externalControlSocketIsOn.getBooleanValue())
            externalControl.startSocket();
         else
            externalControl.stopSocket();
      });

      YoRegistry registryForBlenders = REDUCE_YOVARIABLES ? null : registry;

      for (OneDoFJointBasics controlledJoint : controlledJoints)
      {
         String jointName = controlledJoint.getName();

         YoDouble freezePosition = new YoDouble(jointName + "_frozen_qDesired", registry);
         freezePosition.setToNaN();
         frozenJointDesireds.add(controlledJoint, freezePosition);

         JointControlBlender jointControlBlender = new JointControlBlender("_ExternalBlender", controlledJoint, registryForBlenders);
         jointCommandBlenders.add(controlledJoint, jointControlBlender);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         measuredFootCoPs.put(robotSide, new YoFramePoint3D(robotSide.getShortLowerCaseName() + "_MeasuredFootCoP", ReferenceFrame.getWorldFrame(), registry));
         measuredFootForces.put(robotSide,
                                new YoFrameVector3D(robotSide.getShortLowerCaseName() + "_MeasuredFootForce", ReferenceFrame.getWorldFrame(), registry));
      }

      for (String name : externalControl.debugDataNames)
      {
         debugData.put(name, new YoDouble("zmq_mpc_" + name, registry));
      }
   }

   @Override
   public void onEntry()
   {
      for (int jointIndex = 0; jointIndex < frozenJointDesireds.size(); jointIndex++)
      {
         OneDoFJointBasics joint = frozenJointDesireds.get(jointIndex).getLeft();
         YoDouble setpoint = frozenJointDesireds.get(jointIndex).getRight();
         JointDesiredOutputReadOnly lowLevelJointData = highLevelControllerOutput.getJointDesiredOutput(joint);
         if (lowLevelJointData != null && lowLevelJointData.hasDesiredPosition())
            setpoint.set(lowLevelJointData.getDesiredPosition());
         else
            setpoint.set(joint.getQ());
      }

      blendRatioCurrentValue.set(0.0);

      commandInputManager.clearAllCommands();
      commandInputManager.setEnabled(false);

      lowestFootSide.set(null);
      filteredShouldSwitchSupportSide.set(false);
      filteredRootHeightOffset.set(rootHeightBias.getValue());
   }

   @Override
   public void doAction(double timeInState)
   {
      updateContactState();

      controllerToolbox.update();

      centerOfMass.set(controllerToolbox.getCenterOfMassPosition());
      CapturePointTools.computeCapturePointPosition(controllerToolbox.getCenterOfMassPosition(),
                                                    controllerToolbox.getCenterOfMassVelocity(),
                                                    controllerToolbox.getOmega0(),
                                                    capturePoint);

      totalControllerTime.startMeasurement();
      externalControl.setFootStates(controllerToolbox.getReferenceFrames().getSoleFrames(),
                                    controllerToolbox.getFootSwitches().get(RobotSide.LEFT).hasFootHitGroundFiltered(),
                                    controllerToolbox.getFootSwitches().get(RobotSide.RIGHT).hasFootHitGroundFiltered());
      externalControl.writeRobotState(controllerToolbox.getYoTime().getDoubleValue(),
                                      desiredMode.getOrdinal(),
                                      desiredBehavior.getOrdinal(),
                                      filteredRootHeightOffset.getDoubleValue());

      externalControl.readControlSolution();

      double gainRatio = blendRatioCurrentValue.getValue();

      if (Double.isNaN(gainRatio))
         gainRatio = 0.0;
      else
         gainRatio = MathTools.clamp(gainRatio, 0.0, 1.0);

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointCommandBlenders.get(jointIndex).getLeft();

         YoDouble frozenDesiredPosition = frozenJointDesireds.get(jointIndex).getRight();

         JointDesiredOutputBasics frozenJointData = frozenJointDataHolder.getJointDesiredOutput(joint);
         frozenJointData.clear();
         frozenJointData.setDesiredPosition(frozenDesiredPosition.getDoubleValue());
         frozenJointData.setDesiredVelocity(0.0);
         frozenJointData.setDesiredAcceleration(0.0);
      }
      frozenJointDataHolder.completeWith(getStateSpecificJointSettings());

      for (int jointIndex = 0; jointIndex < jointCommandBlenders.size(); jointIndex++)
      {
         OneDoFJointBasics joint = jointCommandBlenders.get(jointIndex).getLeft();

         JointDesiredOutputBasics externalJointData = externalJointDataHolder.getJointDesiredOutput(joint);
         externalControl.getSolutionData(joint).getJointDesiredOutput(externalJointData);

         JointDesiredOutputBasics frozenJointData = frozenJointDataHolder.getJointDesiredOutput(joint);

         JointDesiredOutputBasics lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(joint);
         lowLevelJointData.clear();

         double localGain = gainRatio;
         if (joint == controllerToolbox.getFullRobotModel().getSpineJoint(SpineJointName.SPINE_ROLL) || joint == controllerToolbox.getFullRobotModel()
                                                                                                                                  .getSpineJoint(SpineJointName.SPINE_PITCH))
         {
            localGain = 0.0;
         }

         JointControlBlender jointControlBlender = jointCommandBlenders.get(jointIndex).getRight();
         jointControlBlender.computeAndUpdateJointControl(lowLevelJointData, frozenJointData, externalJointData, localGain);
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());

      totalControllerTime.stopMeasurement();
      externalControl.readDebugData();

      for (int i = 0; i < externalControl.solutionDebugData.numRows; i++)
      {
         debugData.get(externalControl.debugDataNames[i]).set(externalControl.solutionDebugData.get(i));
      }
   }

   private void updateContactState()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FootSwitchInterface footSwitch = controllerToolbox.getFootSwitches().get(robotSide);
         footSwitch.update();

         if (footSwitch.hasFootHitGroundFiltered())
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
         else
            controllerToolbox.setFootContactStateFree(robotSide);

         measuredFootCoPs.get(robotSide).setMatchingFrame(footSwitch.getCenterOfPressure(), 0.0);
         measuredFootForces.get(robotSide).setMatchingFrame(footSwitch.getMeasuredWrench().getLinearPart());
      }

      // This is used to compute the height offset that may be happening with the robot drifting vertically.
      updateLowestFootSide();
      computeRootHeightOffset();
   }

   @Override
   public void onExit(double timeInState)
   {
      commandInputManager.setEnabled(true);
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }

   private final FramePoint3D lowestFootPosition = new FramePoint3D();

   private void updateLowestFootSide()
   {
      leftFootPosition.setFromReferenceFrame(controllerToolbox.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      rightFootPosition.setFromReferenceFrame(controllerToolbox.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));

      // This is true if we just switched sides. If that's the case, we should reset the switching filter to false.
      if (filteredShouldSwitchSupportSide.getBooleanValue())
         filteredShouldSwitchSupportSide.set(false);

      boolean leftFootInContact = controllerToolbox.getFootSwitches().get(RobotSide.LEFT).hasFootHitGroundFiltered();
      boolean rightFootInContact = controllerToolbox.getFootSwitches().get(RobotSide.RIGHT).hasFootHitGroundFiltered();
      if (leftFootInContact && rightFootInContact)
      {
         // Both feet are in contact. Compute which foot is lower, based on the sole frame.
         boolean leftFootIsLower = leftFootPosition.getZ() < rightFootPosition.getZ();
         RobotSide lowestSide = leftFootIsLower ? RobotSide.LEFT : RobotSide.RIGHT;

         if (lowestFootSide.getValue() == null)
         {
            // The previous lowest foot side hasn't been set. This could be because we just entered the state, or because we just landed from flight. This means
            // we can just hard-set the lowest foot side with no filtering. We should also reset the switching filter to false, since we just switched.
            lowestFootSide.set(lowestSide);
            filteredShouldSwitchSupportSide.set(false);
         }
         else
         {
            // Check if the current lowest side matches the previous lowest side. If not, we should switch sides.
            shouldSwitchSupportSide.set(lowestSide != lowestFootSide.getEnumValue());
            // Apply a glitch filter as to whether we should switch to this new side.
            filteredShouldSwitchSupportSide.update();
            if (filteredShouldSwitchSupportSide.getBooleanValue())
            {
               // The glitch filter says yes! We should switch sides.
               lowestFootSide.set(lowestSide);
            }
         }
      }
      else if (leftFootInContact)
      {
         lowestFootSide.set(RobotSide.LEFT);
      }
      else if (rightFootInContact)
      {
         lowestFootSide.set(RobotSide.RIGHT);
      }
      else
      {
         lowestFootSide.set(null);
      }
   }

   private void computeRootHeightOffset()
   {
      if (lowestFootSide.getValue() == null)
         // If this is null, then neither foot is in contact. We don't need to upset the offset, in that case.
         return;

      // Compute the height of the lowest foot.
      lowestFootPosition.setToZero(controllerToolbox.getReferenceFrames().getSoleFrame(lowestFootSide.getEnumValue()));
      lowestFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

      // We want the height of the lowest foot post-offset to be zero. So the offset is the negative of the height.
      rootHeightOffset.set(-lowestFootPosition.getZ());
      // Apply a low-pass filter to the offset.
      filteredRootHeightOffset.update(rootHeightOffset.getDoubleValue() + rootHeightBias.getDoubleValue());
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("Center of Mass Point",
                                                                    centerOfMass,
                                                                    0.02,
                                                                    ColorDefinitions.Black().darker(),
                                                                    YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_CROSS));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("Capture Point",
                                                                    capturePoint,
                                                                    0.02,
                                                                    ColorDefinitions.Blue().darker(),
                                                                    YoGraphicDefinitionFactory.DefaultPoint2DGraphic.CIRCLE_CROSS));
      for (RobotSide robotSide : RobotSide.values)
      {
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D(robotSide.getUpperCaseName() + " Measured CoP",
                                                                       measuredFootCoPs.get(robotSide),
                                                                       0.008,
                                                                       ColorDefinitions.Black().darker(),
                                                                       YoGraphicDefinitionFactory.DefaultPoint2DGraphic.DIAMOND_FILLED));
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(robotSide.getUpperCaseName() + " Measured Foot Force",
                                                                       measuredFootCoPs.get(robotSide),
                                                                       measuredFootForces.get(robotSide),
                                                                       0.0015,
                                                                       ColorDefinitions.Red().darker()));
      }
      if (group.isEmpty())
         return null;
      return group;
   }
}

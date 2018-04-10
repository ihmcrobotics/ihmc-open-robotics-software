package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepVisualizer;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.idl.RecyclingArrayListPubSub;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.yoVariables.variable.YoInteger;

public class ContinuousStepGenerator implements Updatable
{
   private static final int MAX_NUMBER_OF_FOOTSTEP_TO_VISUALIZE_PER_SIDE = 3;
   public static final Color defaultLeftColor = new Color(0, 205, 102); // springgreen 2, from: http://cloford.com/resources/colours/500col.htm
   public static final Color defaultRightColor = new Color(205, 133, 0); // orange 3, from: http://cloford.com/resources/colours/500col.htm
   public static final SideDependentList<Color> defaultFeetColors = new SideDependentList<Color>(defaultLeftColor, defaultRightColor);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoBoolean walk = new YoBoolean("walk", registry);
   private final YoBoolean walkPreviousValue = new YoBoolean("walkPreviousValue", registry);

   private final YoEnum<RobotSide> currentSupportSide = new YoEnum<>("currentSupportSide", registry, RobotSide.class);
   private final YoFramePoint3D currentSupportFootPosition = new YoFramePoint3D("currentSupportFootPosition", worldFrame, registry);
   private final YoFrameQuaternion currentSupportFootOrientation = new YoFrameQuaternion("currentSupportFootOrientation", worldFrame, registry);

   private final YoInteger numberOfFootstepsToPlan = new YoInteger("numberOfFootstepsToPlan", registry);

   private final YoDouble inPlaceWidth = new YoDouble("footstepGeneratorInPlaceWidth", registry);
   private final YoDouble minStepWidth = new YoDouble("minStepWidth", registry);
   private final YoDouble maxStepWidth = new YoDouble("maxStepWidth", registry);
   private final YoDouble maxStepLength = new YoDouble("maxStepLength", registry);
   private final YoDouble maxAngleTurnOutwards = new YoDouble("maxAngleTurnOutwards", registry);
   private final YoDouble maxAngleTurnInwards = new YoDouble("maxAngleTurnInwards", registry);

   private final YoDouble swingTime = new YoDouble("footstepGeneratorSwingTime", registry);
   private final YoDouble transferTime = new YoDouble("footstepGeneratorTransferTime", registry);
   private final DoubleProvider stepTime = () -> swingTime.getDoubleValue() + transferTime.getDoubleValue();

   private FootPoseProvider footPoseProvider;
   private DesiredVelocityProvider desiredVelocityProvider;
   private DesiredHeadingProvider desiredHeadingProvider;
   private FootstepMessenger footstepMessenger;

   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private final RecyclingArrayListPubSub<FootstepDataMessage> footsteps = footstepDataListMessage.getFootstepDataList();
   private final FootstepDataMessage firstFootstep = new FootstepDataMessage();

   private final SideDependentList<List<FootstepVisualizer>> footstepSideDependentVisualizers = new SideDependentList<>();

   public ContinuousStepGenerator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      parentRegistry.addChild(registry);

      numberOfFootstepsToPlan.set(4);
      minStepWidth.set(0.0);
      maxStepWidth.set(Double.POSITIVE_INFINITY);
      maxStepLength.set(Double.POSITIVE_INFINITY);

      String graphicListName = "FootstepGenerator";

      for (RobotSide robotSide : RobotSide.values)
      {
         List<FootstepVisualizer> visualizers = new ArrayList<>();

         List<Point2D> footPolygon = FootstepVisualizer.createTrapezoidalFootPolygon(0.12, 0.15, 0.25);

         for (int i = 0; i < MAX_NUMBER_OF_FOOTSTEP_TO_VISUALIZE_PER_SIDE; i++)
         {
            String name = robotSide.getCamelCaseNameForStartOfExpression() + "PlannedFootstep" + i;
            AppearanceDefinition footstepColor = new YoAppearanceRGBColor(defaultFeetColors.get(robotSide), 0.0);
            visualizers.add(new FootstepVisualizer(name, graphicListName, robotSide, footPolygon, footstepColor, yoGraphicsListRegistry, registry));
         }

         footstepSideDependentVisualizers.put(robotSide, visualizers);
      }
   }

   private final FramePose2D footstepPose2D = new FramePose2D();
   private final FramePose2D nextFootstepPose2D = new FramePose2D();
   private final FramePose3D nextFootstepPose3D = new FramePose3D();

   private boolean updateFirstFootstep = true;
   /** Hack to slow down the rate at which footsteps are sent. */
   private boolean sendFootsteps = true;

   @Override
   public void update(double time)
   {
      if (!walk.getValue())
      {
         updateFirstFootstep = true;
         return;
      }

      if (walk.getValue() != walkPreviousValue.getValue())
      {
         updateSupportFootPose();
      }

      footstepDataListMessage.setDefaultSwingDuration(Double.NaN);
      footstepDataListMessage.setDefaultTransferDuration(Double.NaN);
      footstepDataListMessage.setFinalTransferDuration(Double.NaN);

      int startIndex = updateFirstFootstep ? 0 : 1;

      RobotSide swingSide;

      if (updateFirstFootstep)
      {
         footsteps.clear();
         footstepPose2D.getPosition().set(currentSupportFootPosition);
         footstepPose2D.getOrientation().set(currentSupportFootOrientation);
         swingSide = currentSupportSide.getEnumValue().getOppositeSide();
      }
      else
      {
         while (footsteps.size() > 1)
            footsteps.fastRemove(footsteps.size() - 1);

         footstepPose2D.getPosition().set(firstFootstep.getLocation());
         footstepPose2D.getOrientation().set(firstFootstep.getOrientation());
         swingSide = RobotSide.fromByte(firstFootstep.getRobotSide()).getOppositeSide();
      }

      for (int i = startIndex; i < numberOfFootstepsToPlan.getValue(); i++)
      {
         Vector2DReadOnly desiredVelocity = desiredVelocityProvider.getDesiredVelocity();

         double xDisplacement = MathTools.clamp(stepTime.getValue() * desiredVelocity.getX(), maxStepLength.getValue());
         double yDisplacement = stepTime.getValue() * desiredVelocity.getY() + swingSide.negateIfRightSide(inPlaceWidth.getValue());
         double headingDisplacement = stepTime.getValue() * desiredHeadingProvider.getHeadingVelocity();

         if (swingSide == RobotSide.LEFT)
         {
            yDisplacement = MathTools.clamp(yDisplacement, minStepWidth.getValue(), maxStepWidth.getValue());
            headingDisplacement = MathTools.clamp(headingDisplacement, maxAngleTurnInwards.getValue(), maxAngleTurnOutwards.getValue());
         }
         else
         {
            yDisplacement = MathTools.clamp(yDisplacement, -maxStepWidth.getValue(), -minStepWidth.getValue());
            headingDisplacement = MathTools.clamp(headingDisplacement, -maxAngleTurnOutwards.getValue(), -maxAngleTurnInwards.getValue());
         }

         nextFootstepPose2D.set(footstepPose2D);
         nextFootstepPose2D.appendTranslation(xDisplacement, yDisplacement);
         nextFootstepPose2D.appendRotation(headingDisplacement);

         nextFootstepPose3D.set(nextFootstepPose2D);
         nextFootstepPose3D.getPosition().setZ(footPoseProvider.getCurrentFootPose(currentSupportSide.getEnumValue()).getZ());

         int vizualizerIndex = i / 2;
         if (vizualizerIndex < MAX_NUMBER_OF_FOOTSTEP_TO_VISUALIZE_PER_SIDE)
         {
            FootstepVisualizer footstepVisualizer = footstepSideDependentVisualizers.get(swingSide).get(vizualizerIndex);
            footstepVisualizer.update(nextFootstepPose3D);
         }

         FootstepDataMessage footstep = footsteps.add();
         footstep.setRobotSide(swingSide.toByte());
         footstep.getLocation().set(nextFootstepPose3D.getPosition());
         footstep.getOrientation().set(nextFootstepPose3D.getOrientation());

         footstepPose2D.set(nextFootstepPose2D);
         swingSide = swingSide.getOppositeSide();
      }

      if (updateFirstFootstep)
      {
         firstFootstep.set(footsteps.get(0));
         updateFirstFootstep = false;
      }

      if (sendFootsteps && footstepMessenger != null)
         footstepMessenger.submitFootsteps(footstepDataListMessage);

      sendFootsteps = !sendFootsteps;
      walkPreviousValue.set(walk.getValue());
   }

   private void updateSupportFootPose()
   {
      FramePose3DReadOnly currentFootPose = footPoseProvider.getCurrentFootPose(currentSupportSide.getEnumValue());
      currentSupportFootPosition.set(currentFootPose.getPosition());
      currentSupportFootOrientation.set(currentFootPose.getOrientation());
   }

   public void setFootstepTiming(double swingTime, double transferTime)
   {
      this.swingTime.set(swingTime);
      this.transferTime.set(transferTime);
   }

   public void setDesiredHeadingProvider(DesiredHeadingProvider desiredHeadingProvider)
   {
      this.desiredHeadingProvider = desiredHeadingProvider;
   }

   public void setDesiredVelocityProvider(DesiredVelocityProvider desiredVelocityProvider)
   {
      this.desiredVelocityProvider = desiredVelocityProvider;
   }

   public void createYoComponentProviders()
   {
      DoubleParameter desiredHeadingVelocity = new DoubleParameter("desiredHeadingVelocity", registry, 0.0);
      YoFrameVector2D desiredVelocity = new YoFrameVector2D("desiredVelocity", ReferenceFrame.getWorldFrame(), registry);
      setDesiredHeadingProvider(() -> desiredHeadingVelocity.getValue());
      setDesiredVelocityProvider(() -> desiredVelocity);
   }

   public void setFootPoseProvider(FootPoseProvider footPoseProvider)
   {
      this.footPoseProvider = footPoseProvider;
   }

   public void createFrameBasedFootPoseProvider(SideDependentList<? extends ReferenceFrame> soleFrames)
   {
      setFootPoseProvider(new FootPoseProvider()
      {
         private final FramePose3D currentFootPose = new FramePose3D();

         @Override
         public FramePose3DReadOnly getCurrentFootPose(RobotSide robotSide)
         {
            currentFootPose.setFromReferenceFrame(soleFrames.get(robotSide));
            return currentFootPose;
         }
      });
   }

   public void notifyFootstepCompleted(RobotSide robotSide)
   {
      updateFirstFootstep = true;
      currentSupportSide.set(robotSide);
      updateSupportFootPose();
   }

   public void notifyFootstepStarted()
   {
      footsteps.remove(0);
   }

   public void createFootstepStatusListener(StatusMessageOutputManager statusMessageOutputManager)
   {
      statusMessageOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, new StatusMessageListener<FootstepStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(FootstepStatusMessage statusMessage)
         {
            FootstepStatus status = FootstepStatus.fromByte(statusMessage.getFootstepStatus());
            if (status == FootstepStatus.COMPLETED)
               notifyFootstepCompleted(RobotSide.fromByte(statusMessage.getRobotSide()));
            else if (status == FootstepStatus.STARTED)
               notifyFootstepStarted();
         }
      });
   }

   public void configureWith(WalkingControllerParameters walkingControllerParameters)
   {
      swingTime.set(walkingControllerParameters.getDefaultSwingTime());
      transferTime.set(walkingControllerParameters.getDefaultTransferTime());
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      inPlaceWidth.set(steppingParameters.getInPlaceWidth());
      minStepWidth.set(steppingParameters.getMinStepWidth());
      maxStepWidth.set(steppingParameters.getMaxStepWidth());
      maxStepLength.set(steppingParameters.getMaxStepLength());
      maxAngleTurnOutwards.set(steppingParameters.getMaxAngleTurnOutwards());
      maxAngleTurnInwards.set(-Math.abs(steppingParameters.getMaxAngleTurnInwards())); // Just to make sure of the sign.
   }

   public void setFootstepMessenger(FootstepMessenger footstepMessenger)
   {
      this.footstepMessenger = footstepMessenger;
   }

   public void startWalking()
   {
      walk.set(true);
   }

   public void stopWalking()
   {
      walk.set(false);
   }
}

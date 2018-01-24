package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class UserDesiredFootstepDataMessageGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final String namePrefix = "userDesiredStep";
   private final YoInteger stepsToTake = new YoInteger(namePrefix + "sToTake", registry);
   private final YoEnum<RobotSide> firstStepSide = new YoEnum<RobotSide>(namePrefix + "FirstSide", registry, RobotSide.class);
   private final YoDouble minimumWidth = new YoDouble(namePrefix + "MinWidth", registry);

   private final YoBoolean stepSquareUp = new YoBoolean(namePrefix + "SquareUp", registry);

   private final YoDouble swingTime = new YoDouble(namePrefix + "SwingTime", registry);
   private final YoDouble transferTime = new YoDouble(namePrefix + "TransferTime", registry);

   private final YoDouble swingHeight = new YoDouble(namePrefix + "SwingHeight", registry);

   private final YoDouble stepHeelPercentage = new YoDouble(namePrefix + "HeelPercentage", registry);
   private final YoDouble stepToePercentage = new YoDouble(namePrefix + "ToePercentage", registry);

   private final YoDouble stepLength = new YoDouble(namePrefix + "Length", registry);
   private final YoDouble stepWidth = new YoDouble(namePrefix + "Width", registry);
   private final YoDouble stepHeight = new YoDouble(namePrefix + "Height", registry);
   private final YoDouble stepSideways = new YoDouble(namePrefix + "Sideways", registry);

   private final YoDouble stepYaw = new YoDouble(namePrefix + "Yaw", registry);
   private final YoDouble stepPitch = new YoDouble(namePrefix + "Pitch", registry);
   private final YoDouble stepRoll = new YoDouble(namePrefix + "Roll", registry);

   private final YoBoolean sendSteps = new YoBoolean(namePrefix + "Send", registry);

   private List<FramePoint2D> contactFramePoints;
   private RecyclingArrayList<Point2D> contactPoints = new RecyclingArrayList<Point2D>(4, Point2D.class);
   private Point2D contactPoint;

   private final SideDependentList<ContactableFoot> bipedFeet;
   private ContactableFoot swingFoot;
   private RobotSide swingSide = RobotSide.LEFT;
   private RobotSide supportSide = swingSide.getOppositeSide();

   private final CommandInputManager commandInputManager;

   private final PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", new FramePose3D());
   private ReferenceFrame newStepReferenceFrame;
   private PoseReferenceFrame previousPoseFrame;

   private FrameVector3D desiredOffset;
   private FramePoint3D desiredPosition;
   private FrameQuaternion desiredOrientation;

   private final FootstepDataCommand desiredFootstepCommand = new FootstepDataCommand();
   private final FootstepDataListCommand footstepCommandList = new FootstepDataListCommand();

   public UserDesiredFootstepDataMessageGenerator(final CommandInputManager commandInputManager, final SideDependentList<ContactableFoot> bipedFeet,
         final WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      this.bipedFeet = bipedFeet;
      this.commandInputManager = commandInputManager;

      swingFoot = bipedFeet.get(swingSide);

      ReferenceFrame stanceFootFrame = bipedFeet.get(swingSide.getOppositeSide()).getSoleFrame();
      desiredOffset = new FrameVector3D(stanceFootFrame);
      desiredPosition = new FramePoint3D(stanceFootFrame);
      desiredOrientation = new FrameQuaternion(stanceFootFrame);

      firstStepSide.set(supportSide);
      minimumWidth.set(walkingControllerParameters.getSteppingParameters().getMinStepWidth());
      stepWidth.set((walkingControllerParameters.getSteppingParameters().getMaxStepWidth()
            + walkingControllerParameters.getSteppingParameters().getMinStepWidth()) / 2);
      swingHeight.set(0.0);

      swingTime.set(walkingControllerParameters.getDefaultSwingTime());
      transferTime.set(walkingControllerParameters.getDefaultTransferTime());

      stepHeelPercentage.set(1.0);
      stepToePercentage.set(1.0);

      sendSteps.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (sendSteps.getBooleanValue())
            {
               createStepPlan();
            }
         }
      });

      parentRegistry.addChild(registry);
   }

   public void createStepPlan()
   {
      desiredFootstepCommand.clear();
      footstepCommandList.clear();

      sendSteps.set(false);

      swingSide = firstStepSide.getEnumValue();
      if (swingSide == null)
         swingSide = RobotSide.LEFT;

      supportSide = swingSide.getOppositeSide();
      previousPoseFrame = null;

      for (int i = 0; i < stepsToTake.getIntegerValue(); i++)
      {
         swingFoot = bipedFeet.get(swingSide);

         desiredFootstepCommand.clear();
         desiredFootstepCommand.setRobotSide(swingSide);
         desiredFootstepCommand.setSwingHeight(swingHeight.getDoubleValue());

         if ((i == stepsToTake.getIntegerValue()-1) && stepSquareUp.getBooleanValue())
         {
            squareUp();
         }
         else
         {
            createNextFootstep();
         }

         footstepCommandList.addFootstep(desiredFootstepCommand);

         previousPoseFrame = footstepPoseFrame;
         swingSide = swingSide.getOppositeSide();
         supportSide = swingSide.getOppositeSide();
      }

      footstepCommandList.setExecutionMode(ExecutionMode.OVERRIDE);
      footstepCommandList.setDefaultSwingDuration(swingTime.getDoubleValue());
      footstepCommandList.setDefaultTransferDuration(transferTime.getDoubleValue());

      commandInputManager.submitCommand(footstepCommandList);
   }

   private void createNextFootstep()
   {
      createFootstep();
   }

   private void squareUp()
   {
      createFootstep(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   private void createFootstep()
   {
      createFootstep(stepLength.getDoubleValue(), stepSideways.getDoubleValue(), stepHeight.getDoubleValue(), stepYaw.getDoubleValue(),
            stepPitch.getDoubleValue(), stepRoll.getDoubleValue());
   }


   private void createFootstep(double stepLength, double stepSideways, double stepHeight, double stepYaw, double stepPitch, double stepRoll)
   {
      if (previousPoseFrame != null)
      {
         newStepReferenceFrame = previousPoseFrame;
      }
      else
      {
         newStepReferenceFrame = bipedFeet.get(supportSide).getSoleFrame();
      }

      // Footstep Position
      desiredPosition.setToZero(newStepReferenceFrame);
      desiredOffset.setToZero(newStepReferenceFrame);

      double stepYOffset = supportSide.negateIfLeftSide(stepWidth.getDoubleValue()) + stepSideways;
      if ((supportSide == RobotSide.LEFT) && (stepYOffset > -minimumWidth.getDoubleValue()))
      {
         stepYOffset = -minimumWidth.getDoubleValue();
      }
      if ((supportSide == RobotSide.RIGHT) && (stepYOffset < minimumWidth.getDoubleValue()))
      {
         stepYOffset = minimumWidth.getDoubleValue();
      }
      desiredOffset.set(stepLength, stepYOffset, stepHeight);
      desiredPosition.add(desiredOffset);
      desiredPosition.changeFrame(worldFrame);

      // Footstep orientation
      desiredOrientation.setToZero(newStepReferenceFrame);
      desiredOrientation.setYawPitchRoll(stepYaw, 0.0, 0.0);
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.setYawPitchRoll(desiredOrientation.getYaw(), stepPitch, stepRoll);

      footstepPoseFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);
      desiredFootstepCommand.setPose(desiredPosition, desiredOrientation);

      // set contact points
      contactFramePoints = swingFoot.getContactPoints2d();
      contactPoints.clear();
      for (FramePoint2D contactFramePoint : contactFramePoints)
      {
         contactPoint = new Point2D(contactFramePoint);

         if (contactFramePoint.getX() > 0.0)
            contactPoint.setX(contactPoint.getX() * stepToePercentage.getDoubleValue());
         else
            contactPoint.setX(contactPoint.getX() * stepHeelPercentage.getDoubleValue());

         contactPoints.add().set(contactPoint);
      }
      desiredFootstepCommand.setPredictedContactPoints(contactPoints);
   }
}

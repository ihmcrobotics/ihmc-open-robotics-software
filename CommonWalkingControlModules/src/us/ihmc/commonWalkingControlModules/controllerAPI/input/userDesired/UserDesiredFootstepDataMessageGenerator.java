package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class UserDesiredFootstepDataMessageGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final String namePrefix = "userDesiredStep";
   private final IntegerYoVariable stepsToTake = new IntegerYoVariable(namePrefix + "sToTake", registry);
   private final EnumYoVariable<RobotSide> firstStepSide = new EnumYoVariable<RobotSide>(namePrefix + "FirstSide", registry, RobotSide.class);
   private final DoubleYoVariable minimumWidth = new DoubleYoVariable(namePrefix + "MinWidth", registry);

   private final BooleanYoVariable stepSquareUp = new BooleanYoVariable(namePrefix + "SquareUp", registry);

   private final DoubleYoVariable swingTime = new DoubleYoVariable(namePrefix + "SwingTime", registry);
   private final DoubleYoVariable transferTime = new DoubleYoVariable(namePrefix + "TransferTime", registry);

   private final DoubleYoVariable swingHeight = new DoubleYoVariable(namePrefix + "SwingHeight", registry);

   private final DoubleYoVariable stepHeelPercentage = new DoubleYoVariable(namePrefix + "HeelPercentage", registry);
   private final DoubleYoVariable stepToePercentage = new DoubleYoVariable(namePrefix + "ToePercentage", registry);

   private final DoubleYoVariable stepLength = new DoubleYoVariable(namePrefix + "Length", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable(namePrefix + "Width", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable(namePrefix + "Height", registry);
   private final DoubleYoVariable stepSideways = new DoubleYoVariable(namePrefix + "Sideways", registry);

   private final DoubleYoVariable stepYaw = new DoubleYoVariable(namePrefix + "Yaw", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable(namePrefix + "Pitch", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable(namePrefix + "Roll", registry);

   private final BooleanYoVariable sendSteps = new BooleanYoVariable(namePrefix + "Send", registry);

   private List<FramePoint2d> contactFramePoints;
   private RecyclingArrayList<Point2d> contactPoints = new RecyclingArrayList<Point2d>(4, Point2d.class);
   private Point2d contactPoint;

   private final SideDependentList<ContactableFoot> bipedFeet;
   private ContactableFoot swingFoot;
   private RobotSide swingSide = RobotSide.LEFT;
   private RobotSide supportSide = swingSide.getOppositeSide();

   private final CommandInputManager commandInputManager;

   private final PoseReferenceFrame footstepPoseFrame = new PoseReferenceFrame("footstepPoseFrame", new FramePose());
   private ReferenceFrame newStepReferenceFrame;
   private PoseReferenceFrame previousPoseFrame;

   private FrameVector desiredOffset;
   private FramePoint desiredPosition;
   private FrameOrientation desiredOrientation;

   private final FootstepDataCommand desiredFootstepCommand = new FootstepDataCommand();
   private final FootstepDataListCommand footstepCommandList = new FootstepDataListCommand();

   public UserDesiredFootstepDataMessageGenerator(final CommandInputManager commandInputManager, final SideDependentList<ContactableFoot> bipedFeet,
         final WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
   {
      this.bipedFeet = bipedFeet;
      this.commandInputManager = commandInputManager;

      swingFoot = bipedFeet.get(swingSide);

      ReferenceFrame stanceFootFrame = bipedFeet.get(swingSide.getOppositeSide()).getSoleFrame();
      desiredOffset = new FrameVector(stanceFootFrame);
      desiredPosition = new FramePoint(stanceFootFrame);
      desiredOrientation = new FrameOrientation(stanceFootFrame);

      firstStepSide.set(supportSide);
      minimumWidth.set(walkingControllerParameters.getMinStepWidth());
      stepWidth.set((walkingControllerParameters.getMaxStepWidth() + walkingControllerParameters.getMinStepWidth()) / 2);
      swingHeight.set(0.0);

      swingTime.set(walkingControllerParameters.getDefaultSwingTime());
      transferTime.set(walkingControllerParameters.getDefaultTransferTime());

      stepHeelPercentage.set(1.0);
      stepToePercentage.set(1.0);

      sendSteps.addVariableChangedListener(new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
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
         desiredFootstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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
      footstepCommandList.setSwingTime(swingTime.getDoubleValue());
      footstepCommandList.setTransferTime(transferTime.getDoubleValue());

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
      desiredFootstepCommand.setPose(desiredPosition.getPoint(), desiredOrientation.getQuaternion());

      // set contact points
      contactFramePoints = swingFoot.getContactPoints2d();
      contactPoints.clear();
      for (FramePoint2d contactFramePoint : contactFramePoints)
      {
         contactPoint = contactFramePoint.getPointCopy();

         if (contactFramePoint.getX() > 0.0)
            contactPoint.setX(contactPoint.getX() * stepToePercentage.getDoubleValue());
         else
            contactPoint.setX(contactPoint.getX() * stepHeelPercentage.getDoubleValue());

         contactPoints.add().set(contactPoint);
      }
      desiredFootstepCommand.setPredictedContactPoints(contactPoints);
   }
}

package us.ihmc.commonWalkingControlModules.controllerAPI.input.userDesired;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import javax.vecmath.Point2d;
import java.util.List;

public class UserDesiredFootstepDataMessageGenerator implements Updatable
{
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

   private final SideDependentList<ContactableFoot> bipedFeet;
   private ContactableFoot swingFoot;
   private RobotSide swingSide = RobotSide.LEFT;
   private RobotSide supportSide = swingSide.getOppositeSide();

   private final CommandInputManager commandInputManager;

   private ReferenceFrame newStepReferenceFrame;
   private FrameVector desiredOffset;
   private FramePoint desiredPosition;
   private FramePoint previousPosition;
   private FrameOrientation desiredOrientation;
   private FrameOrientation previousOrientation;

   private final FootstepDataControllerCommand desiredFootstepCommand = new FootstepDataControllerCommand();
   private final FootstepDataListCommand footstepCommandList = new FootstepDataListCommand();

   public UserDesiredFootstepDataMessageGenerator(SideDependentList<ContactableFoot> bipedFeet, CommandInputManager commandInputManager,
         WalkingControllerParameters walkingControllerParameters, YoVariableRegistry parentRegistry)
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

      parentRegistry.addChild(registry);
   }

   public void update(double time)
   {
      desiredFootstepCommand.clear();
      footstepCommandList.clear();

      if (sendSteps.getBooleanValue())
      {
         sendSteps.set(false);

         swingSide = firstStepSide.getEnumValue();
         if (swingSide == null)
            swingSide = RobotSide.LEFT;

         supportSide = swingSide.getOppositeSide();
         previousPosition = null;
         previousOrientation = null;

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

            previousPosition = desiredPosition;
            previousOrientation = desiredOrientation;
            swingSide = swingSide.getOppositeSide();
            supportSide = swingSide.getOppositeSide();
         }

         footstepCommandList.setSwingTime(swingTime.getDoubleValue());
         footstepCommandList.setTransferTime(transferTime.getDoubleValue());

         commandInputManager.submitCommand(footstepCommandList);
      }
   }

   private void createNextFootstep()
   {
      if (previousPosition != null)
      {
         newStepReferenceFrame = previousPosition.getReferenceFrame();
      }
      else
      {
         newStepReferenceFrame = bipedFeet.get(supportSide).getSoleFrame();
      }

      createFootstep();
   }

   private void squareUp()
   {
      if (previousPosition != null)
      {
         newStepReferenceFrame = previousPosition.getReferenceFrame();
      }
      else
      {
         newStepReferenceFrame = bipedFeet.get(supportSide).getSoleFrame();
      }

      createFootstep(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   private void createFootstep()
   {
      createFootstep(stepLength.getDoubleValue(), stepSideways.getDoubleValue(), stepHeight.getDoubleValue(), stepYaw.getDoubleValue(),
            stepPitch.getDoubleValue(), stepRoll.getDoubleValue());
   }

   private List<FramePoint2d> contactFramePoints;
   private RecyclingArrayList<Point2d> contactPoints = new RecyclingArrayList<Point2d>(4, Point2d.class);
   private Point2d contactPoint;

   private void createFootstep(double stepLength, double stepSideways, double stepHeight, double stepYaw, double stepPitch, double stepRoll)
   {
      // Footstep Position
      desiredPosition.setToZero(newStepReferenceFrame);
      double stepYOffset = supportSide.negateIfLeftSide(stepWidth.getDoubleValue()) + stepSideways;
      if ((supportSide == RobotSide.LEFT) && (stepYOffset > -minimumWidth.getDoubleValue()))
      {
         stepYOffset = -minimumWidth.getDoubleValue();
      }
      if ((supportSide == RobotSide.RIGHT) && (stepYOffset < minimumWidth.getDoubleValue()))
      {
         stepYOffset = minimumWidth.getDoubleValue();
      }
      desiredOffset.setToZero(newStepReferenceFrame);
      desiredOffset.set(stepLength, stepYOffset, stepHeight);
      desiredPosition.add(desiredOffset);

      // Footstep Orientation
      desiredOrientation.setToZero(newStepReferenceFrame);
      desiredOrientation.setYawPitchRoll(stepYaw, stepPitch, stepRoll);

      contactFramePoints = swingFoot.getContactPoints2d();
      contactPoints.clear();

      for (FramePoint2d contactFramePoint : contactFramePoints)
      {
         contactPoint = contactFramePoint.getPointCopy();

         if (contactFramePoint.getX() > 0.0)
            contactPoint.setX(contactPoint.getX() * stepToePercentage.getDoubleValue());
         else
            contactPoint.setX(contactPoint.getX() * stepHeelPercentage.getDoubleValue());

         contactPoints.add(contactPoint);
      }

      desiredFootstepCommand.setPose(desiredPosition.getPoint(), desiredOrientation.getQuaternion());
      desiredFootstepCommand.setPredictedContactPoints(contactPoints);
   }
}

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataMessage;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class AbstractDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final SideDependentList<YoFramePoint> footstepPositions = new SideDependentList<>();
   protected final SideDependentList<YoFrameQuaternion> footstepOrientations = new SideDependentList<>();

   protected final FramePoint framePosition = new FramePoint();
   protected final FrameOrientation frameOrientation = new FrameOrientation();

   public AbstractDesiredFootstepCalculator(YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression() + "Footstep";

         YoFramePoint footstepPosition = new YoFramePoint(namePrefix + "Position", worldFrame, registry);
         footstepPositions.put(robotSide, footstepPosition);

         YoFrameQuaternion footstepOrientation = new YoFrameQuaternion(namePrefix + "Orientation", "", worldFrame, registry);
         footstepOrientations.put(robotSide, footstepOrientation);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public ModifiableFootstepDataMessage updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      footstepPositions.get(swingLegSide).getFrameTupleIncludingFrame(framePosition);
      footstepOrientations.get(swingLegSide).getFrameOrientationIncludingFrame(frameOrientation);

      framePosition.changeFrame(worldFrame);
      frameOrientation.changeFrame(worldFrame);

      ModifiableFootstepDataMessage footstep = new ModifiableFootstepDataMessage();
      footstep.setRobotSide(swingLegSide);
      footstep.setPose(framePosition.getPoint(), frameOrientation.getQuaternion());

      return footstep;
   }
}

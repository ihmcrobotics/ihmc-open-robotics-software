package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class AbstractDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final SideDependentList<YoFramePoint> footstepPositions = new SideDependentList<>();
   protected final SideDependentList<YoFrameQuaternion> footstepOrientations = new SideDependentList<>();

   protected final FramePoint3D framePosition = new FramePoint3D();
   protected final FrameQuaternion frameOrientation = new FrameQuaternion();

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
   public FootstepDataMessage updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      footstepPositions.get(swingLegSide).getFrameTupleIncludingFrame(framePosition);
      footstepOrientations.get(swingLegSide).getFrameOrientationIncludingFrame(frameOrientation);

      framePosition.changeFrame(worldFrame);
      frameOrientation.changeFrame(worldFrame);

      FootstepDataMessage footstep = new FootstepDataMessage();
      footstep.setRobotSide(swingLegSide);
      footstep.setLocation(framePosition.getPoint());
      footstep.setOrientation(frameOrientation.getQuaternion());

      return footstep;
   }
}

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;

public abstract class AbstractDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final SideDependentList<YoFramePoint3D> footstepPositions = new SideDependentList<>();
   protected final SideDependentList<YoFrameQuaternion> footstepOrientations = new SideDependentList<>();

   protected final FramePoint3D framePosition = new FramePoint3D();
   protected final FrameQuaternion frameOrientation = new FrameQuaternion();

   public AbstractDesiredFootstepCalculator(YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression() + "Footstep";

         YoFramePoint3D footstepPosition = new YoFramePoint3D(namePrefix + "Position", worldFrame, registry);
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

      framePosition.setIncludingFrame(footstepPositions.get(swingLegSide));
      frameOrientation.setIncludingFrame(footstepOrientations.get(swingLegSide));

      framePosition.changeFrame(worldFrame);
      frameOrientation.changeFrame(worldFrame);

      FootstepDataMessage footstep = new FootstepDataMessage();
      footstep.setRobotSide(swingLegSide.toByte());
      footstep.getLocation().set(framePosition);
      footstep.getOrientation().set(frameOrientation);

      return footstep;
   }
}

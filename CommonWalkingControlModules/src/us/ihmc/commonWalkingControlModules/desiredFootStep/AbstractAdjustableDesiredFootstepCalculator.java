package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public abstract class AbstractAdjustableDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final SideDependentList<YoFramePoint> footstepPositions = new SideDependentList<YoFramePoint>();
   protected final SideDependentList<YoFrameOrientation> footstepOrientations = new SideDependentList<YoFrameOrientation>();

   private DesiredFootstepAdjustor desiredFootstepAdjustor;

   public AbstractAdjustableDesiredFootstepCalculator(SideDependentList<ReferenceFrame> framesToSaveFootstepIn, YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         String namePrefix = robotSide.getCamelCaseNameForMiddleOfExpression() + "Footstep";

         ReferenceFrame frameToSaveFootstepIn = framesToSaveFootstepIn.get(robotSide);
         YoFramePoint footstepPosition = new YoFramePoint(namePrefix + "Position", frameToSaveFootstepIn, registry);
         footstepPositions.put(robotSide, footstepPosition);

         YoFrameOrientation footstepOrientation = new YoFrameOrientation(namePrefix + "Orientation", "", frameToSaveFootstepIn, registry);
         footstepOrientations.put(robotSide, footstepOrientation);
      }

      parentRegistry.addChild(registry);
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      FramePose footstepPose = new FramePose(footstepPositions.get(swingLegSide).getFramePointCopy(),
                                  footstepOrientations.get(swingLegSide).getFrameOrientationCopy());
      Footstep desiredFootstep = new Footstep(swingLegSide, footstepPose);

      if (desiredFootstepAdjustor != null)
      {
         return desiredFootstepAdjustor.adjustDesiredFootstep(desiredFootstep);
      }

      return desiredFootstep;
   }

   public void setDesiredFootstepAdjustor(DesiredFootstepAdjustor desiredFootstepAdjustor)
   {
      this.desiredFootstepAdjustor = desiredFootstepAdjustor;
   }
}

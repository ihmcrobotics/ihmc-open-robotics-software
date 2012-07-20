
//A Desired Footstep is always defined in the support foot frame

package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class SimpleWorldDesiredFootstepCalculator implements DesiredFootstepCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleFootstepCalculator");

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;

   private final DoubleYoVariable stepLength = new DoubleYoVariable("stepLength", registry);
   private final DoubleYoVariable stepWidth = new DoubleYoVariable("stepWidth", registry);
   private final DoubleYoVariable stepHeight = new DoubleYoVariable("stepHeight", registry);
   private final DoubleYoVariable stepYaw = new DoubleYoVariable("stepYaw", registry);
   private final DoubleYoVariable stepPitch = new DoubleYoVariable("stepPitch", registry);
   private final DoubleYoVariable stepRoll = new DoubleYoVariable("stepRoll", registry);
   private final YoFramePoint previousFootstepPosition = new YoFramePoint("previousFootstep", ReferenceFrame.getWorldFrame(), registry);
   private final double ankleHeight;

   public SimpleWorldDesiredFootstepCalculator(SideDependentList<ReferenceFrame> ankleZUpFrames, DesiredHeadingControlModule desiredHeadingControlModule, YoVariableRegistry parentRegistry, double ankleHeight)
   {
      this.ankleZUpFrames = ankleZUpFrames;
      this.desiredHeadingControlModule = desiredHeadingControlModule;
      this.ankleHeight = ankleHeight;
      previousFootstepPosition.setToNaN();
      parentRegistry.addChild(registry);
   }

   public void initializeDesiredFootstep(RobotSide supportLegSide)
   {
      if (previousFootstepPosition.containsNaN())
      {
         FramePoint supportAnklePosition = new FramePoint(ankleZUpFrames.get(supportLegSide));
         supportAnklePosition.changeFrame(previousFootstepPosition.getReferenceFrame());
         previousFootstepPosition.set(supportAnklePosition);
         previousFootstepPosition.setZ(stepHeight.getDoubleValue() + ankleHeight + 0.01); // TODO: Darpa hack
      }
      else
      {
         RobotSide oldSupportLegSide = supportLegSide.getOppositeSide();
         Footstep footstep = updateAndGetDesiredFootstep(oldSupportLegSide);
         FramePoint footstepPosition = footstep.getFootstepPositionInFrame(previousFootstepPosition.getReferenceFrame());
         previousFootstepPosition.set(footstepPosition);
      }
   }

   public Footstep updateAndGetDesiredFootstep(RobotSide supportLegSide)
   {
      RobotSide swingLegSide = supportLegSide.getOppositeSide();

      // Footstep Frame
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      // Footstep Position
      FramePoint footstepPosition = previousFootstepPosition.getFramePointCopy();
      FrameVector footstepOffset = new FrameVector(desiredHeadingFrame, stepLength.getDoubleValue(), supportLegSide.negateIfLeftSide(stepWidth.getDoubleValue()), stepHeight.getDoubleValue());
      
      footstepPosition.changeFrame(desiredHeadingFrame);
//      footstepOffset.changeFrame(supportAnkleZUpFrame);
      footstepPosition.add(footstepOffset); 

      // Footstep Orientation
      Orientation footstepOrientation = new Orientation(desiredHeadingFrame); 
      footstepOrientation.setYawPitchRoll(stepYaw.getDoubleValue(), stepPitch.getDoubleValue(), stepRoll.getDoubleValue());
//      footstepOrientation.changeFrame(supportAnkleZUpFrame);
      
      // Create a foot Step Pose from Position and Orientation
      FramePose footstepPose = new FramePose(footstepPosition, footstepOrientation);
      Footstep desiredFootstep = new Footstep(swingLegSide, footstepPose);
      
      return desiredFootstep;
   }

   public void setupParametersForM2V2()
   {
      stepLength.set(0.32);
      stepWidth.set(0.22);
      stepHeight.set(0.0);
      stepYaw.set(0.0);
      stepPitch.set(-0.35);
      stepRoll.set(0.0);
   }

   public void setupParametersForR2()
   {
      stepLength.set(0.6);
      stepWidth.set(0.35);
      stepHeight.set(0.0);
      stepYaw.set(0.0);
      stepPitch.set(-0.25);
      stepRoll.set(0.0);
   }
   
   public void setupParametersForR2InverseDynamics()
   {
      // stairs:
      stepLength.set(0.3);
//      stepLength.set(0.21);
      stepWidth.set(0.2);
      stepHeight.set(0.25);
      stepYaw.set(0.0);
      stepPitch.set(0.0);
      stepRoll.set(0.0);
      
      // flat ground
//      stepLength.set(0.35);
//      stepWidth.set(0.2);
//      stepHeight.set(0.0);
//      stepYaw.set(0.0);
//      stepPitch.set(0.0);
//      stepRoll.set(0.0);
   }
}

package us.ihmc.commonWalkingControlModules.kinematics;

import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class BodyPositionEstimatorThroughStanceLeg
{
   private final String name;
   private final YoVariableRegistry registry;
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final RobotSide robotSide;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final YoFramePoint anklePositionFix;
   private final FramePoint bodyPosition = new FramePoint(world);

   public BodyPositionEstimatorThroughStanceLeg(RobotSide robotSide, CommonWalkingReferenceFrames referenceFrames)
   {
      this.robotSide = robotSide;
      this.name = robotSide + getClass().getSimpleName();
      this.registry = new YoVariableRegistry(name);
      this.anklePositionFix = new YoFramePoint("anklePositionFix", "", world, registry);
      this.referenceFrames = referenceFrames;
   }
   
   private final FrameVector tempBodyPositionVector = new FrameVector(world);
   public void computeBodyPosition()
   {
      bodyPosition.setToZero(referenceFrames.getIMUFrame());
      bodyPosition.changeFrame(referenceFrames.getAnkleZUpFrame(robotSide));
      tempBodyPositionVector.setAndChangeFrame(bodyPosition);
      tempBodyPositionVector.changeFrame(world);
      anklePositionFix.getFramePoint(bodyPosition);
      bodyPosition.add(tempBodyPositionVector);
   }
   
   public void packBodyPosition(FramePoint bodyPositionToPack)
   {
      bodyPositionToPack.set(bodyPosition);
   }
   
   public void fixAnklePositionInWorld()
   {
      FramePoint ankle = new FramePoint(referenceFrames.getAnkleZUpFrame(robotSide));
      ankle.changeFrame(world);
      anklePositionFix.set(ankle);
   }
}

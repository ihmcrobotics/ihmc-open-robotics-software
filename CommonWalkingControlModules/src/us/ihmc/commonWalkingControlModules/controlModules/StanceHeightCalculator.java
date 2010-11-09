package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.SideDependentList;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class StanceHeightCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("stanceHeight");
   private final SideDependentList<DoubleYoVariable> stanceHeight = new SideDependentList<DoubleYoVariable>();
   private final SideDependentList<ReferenceFrame> footFrames;
   private final ReferenceFrame upperBodyFrame;

   public StanceHeightCalculator(SideDependentList<ReferenceFrame> footFrames, ReferenceFrame upperBodyFrame, YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         DoubleYoVariable doubleYoVariable = new DoubleYoVariable("stanceHeight" + robotSide.getCamelCaseNameForMiddleOfExpression(), registry);
         stanceHeight.put(robotSide, doubleYoVariable);
      }
      
      this.footFrames = new SideDependentList<ReferenceFrame>(footFrames);
      this.upperBodyFrame = upperBodyFrame;

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }


   public double getStanceHeightUsingOneFoot(RobotSide sideToGetStanceHeightFor)
   {
      FramePoint footOriginPosition = new FramePoint(footFrames.get(sideToGetStanceHeightFor));
      footOriginPosition = footOriginPosition.changeFrameCopy(upperBodyFrame);

      // Now make a vector of it:
      FrameVector upperBodyToFoot = new FrameVector(footOriginPosition);

      // Get Z
      upperBodyToFoot = upperBodyToFoot.changeFrameCopy(ReferenceFrame.getWorldFrame());

      double ret = -upperBodyToFoot.getZ();
      stanceHeight.get(sideToGetStanceHeightFor).set(ret);

      return ret;
   }

   public double getStanceHeightUsingBothFeet()
   {
      double sum = 0.0;
      for (RobotSide robotSide : RobotSide.values())
      {
         sum += getStanceHeightUsingOneFoot(robotSide);
      }

      return sum / RobotSide.values().length;
   }
}

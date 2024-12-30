package us.ihmc.perception.gpuHeightMap;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootBasedDefaultHeightProvider implements DefaultHeightProvider
{
   private static final double thicknessOfTheFoot = 0.02;

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   public FootBasedDefaultHeightProvider(ReferenceFrame leftSoleFrame, ReferenceFrame rightSoleFrame)
   {
      this.soleFrames.put(RobotSide.LEFT, leftSoleFrame);
      this.soleFrames.put(RobotSide.RIGHT, rightSoleFrame);

      if (soleFrames.sides().length != 2)
         throw new RuntimeException("Wasn't provided two valid sole frames");
   }

   @Override
   public double computeDefaultHeight()
   {
      return Math.min(soleFrames.get(RobotSide.LEFT).getTransformToWorldFrame().getTranslationZ(),
                           soleFrames.get(RobotSide.RIGHT).getTransformToWorldFrame().getTranslationZ()) - thicknessOfTheFoot;
   }
}

package us.ihmc.avatar;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class AvatarTestYoVariables
{
   private final DoubleYoVariable yoTime;
   
   private final DoubleYoVariable pelvisX;
   private final DoubleYoVariable pelvisY;
   private final DoubleYoVariable pelvisZ;
   private final DoubleYoVariable pelvisYaw;
   
   private final DoubleYoVariable midFeetZUpZ;
   private final DoubleYoVariable desiredCOMHeight;
   
   private final SideDependentList<YoFramePoint> solePositions = new SideDependentList<>();

   public AvatarTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (DoubleYoVariable) scs.getVariable("t");

      pelvisX = (DoubleYoVariable) scs.getVariable("q_x");
      pelvisY = (DoubleYoVariable) scs.getVariable("q_y");
      pelvisZ = (DoubleYoVariable) scs.getVariable("q_z");
      pelvisYaw = (DoubleYoVariable) scs.getVariable("q_yaw");
      
      midFeetZUpZ = (DoubleYoVariable) scs.getVariable("midFeetZUpZ");
      desiredCOMHeight = (DoubleYoVariable) scs.getVariable("desiredCOMHeight");

      solePositions.set(RobotSide.LEFT, new YoFramePoint((DoubleYoVariable) scs.getVariable("leftSoleX"), (DoubleYoVariable) scs.getVariable("leftSoleY"),
                                                         (DoubleYoVariable) scs.getVariable("leftSoleZ"), ReferenceFrame.getWorldFrame()));
      solePositions.set(RobotSide.RIGHT, new YoFramePoint((DoubleYoVariable) scs.getVariable("rightSoleX"), (DoubleYoVariable) scs.getVariable("rightSoleY"),
                                                         (DoubleYoVariable) scs.getVariable("rightSoleZ"), ReferenceFrame.getWorldFrame()));
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTime;
   }

   public DoubleYoVariable getPelvisX()
   {
      return pelvisX;
   }

   public DoubleYoVariable getPelvisY()
   {
      return pelvisY;
   }

   public DoubleYoVariable getPelvisZ()
   {
      return pelvisZ;
   }

   public DoubleYoVariable getPelvisYaw()
   {
      return pelvisYaw;
   }

   public DoubleYoVariable getMidFeetZUpZ()
   {
      return midFeetZUpZ;
   }

   public DoubleYoVariable getDesiredCOMHeight()
   {
      return desiredCOMHeight;
   }
   
   public YoFramePoint getSolePosition(RobotSide robotSide)
   {
      return solePositions.get(robotSide);
   }
}

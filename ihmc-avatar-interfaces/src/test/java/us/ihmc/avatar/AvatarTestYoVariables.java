package us.ihmc.avatar;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public abstract class AvatarTestYoVariables
{
   private final YoDouble yoTime;
   
   private final YoDouble pelvisX;
   private final YoDouble pelvisY;
   private final YoDouble pelvisZ;
   private final YoDouble pelvisYaw;
   
   private final YoDouble midFeetZUpZ;
   private final YoDouble desiredCOMHeight;
   
   private final SideDependentList<YoFramePoint> solePositions = new SideDependentList<>();

   public AvatarTestYoVariables(SimulationConstructionSet scs)
   {
      yoTime = (YoDouble) scs.getVariable("t");

      pelvisX = (YoDouble) scs.getVariable("q_x");
      pelvisY = (YoDouble) scs.getVariable("q_y");
      pelvisZ = (YoDouble) scs.getVariable("q_z");
      pelvisYaw = (YoDouble) scs.getVariable("q_yaw");
      
      midFeetZUpZ = (YoDouble) scs.getVariable("midFeetZUpZ");
      desiredCOMHeight = (YoDouble) scs.getVariable("desiredCOMHeight");

      solePositions.set(RobotSide.LEFT, new YoFramePoint((YoDouble) scs.getVariable("leftSoleX"), (YoDouble) scs.getVariable("leftSoleY"),
                                                         (YoDouble) scs.getVariable("leftSoleZ"), ReferenceFrame.getWorldFrame()));
      solePositions.set(RobotSide.RIGHT, new YoFramePoint((YoDouble) scs.getVariable("rightSoleX"), (YoDouble) scs.getVariable("rightSoleY"),
                                                         (YoDouble) scs.getVariable("rightSoleZ"), ReferenceFrame.getWorldFrame()));
   }

   public YoDouble getYoTime()
   {
      return yoTime;
   }

   public YoDouble getPelvisX()
   {
      return pelvisX;
   }

   public YoDouble getPelvisY()
   {
      return pelvisY;
   }

   public YoDouble getPelvisZ()
   {
      return pelvisZ;
   }

   public YoDouble getPelvisYaw()
   {
      return pelvisYaw;
   }

   public YoDouble getMidFeetZUpZ()
   {
      return midFeetZUpZ;
   }

   public YoDouble getDesiredCOMHeight()
   {
      return desiredCOMHeight;
   }
   
   public YoFramePoint getSolePosition(RobotSide robotSide)
   {
      return solePositions.get(robotSide);
   }
}

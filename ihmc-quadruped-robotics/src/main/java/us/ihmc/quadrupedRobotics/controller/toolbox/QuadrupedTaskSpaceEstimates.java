package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceEstimates
{
   private final FramePoint3D comPosition = new FramePoint3D();
   private final QuadrantDependentList<FramePoint3D> solePosition = new QuadrantDependentList<>();

   public QuadrupedTaskSpaceEstimates()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void set(QuadrupedTaskSpaceEstimates other)
   {
      this.comPosition.setIncludingFrame(other.comPosition);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.solePosition.get(robotQuadrant).setIncludingFrame(other.solePosition.get(robotQuadrant));
      }
   }

   public FramePoint3D getSolePosition(RobotQuadrant robotQuadrant)
   {
      return solePosition.get(robotQuadrant);
   }

   public QuadrantDependentList<FramePoint3D> getSolePositions()
   {
      return solePosition;
   }


}

package us.ihmc.commonWalkingControlModules.controlModules.foot.wobble;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootRotationInformation
{
   private final SideDependentList<MutableBoolean> isRotating = new SideDependentList<MutableBoolean>(new MutableBoolean(), new MutableBoolean());
   private final SideDependentList<MutableBoolean> hasAdjusted = new SideDependentList<MutableBoolean>(new MutableBoolean(), new MutableBoolean());
   private final SideDependentList<FramePoint3DBasics> desiredCoPs = new SideDependentList<FramePoint3DBasics>(new FramePoint3D(), new FramePoint3D());

   public void setRotating(RobotSide side, FramePoint3DReadOnly desiredCop)
   {
      isRotating.get(side).setTrue();
      desiredCoPs.get(side).setIncludingFrame(desiredCop);
   }

   public void setAdjusted(RobotSide side)
   {
      hasAdjusted.get(side).setTrue();
   }

   public boolean hasAdjusted(RobotSide side)
   {
      return hasAdjusted.get(side).booleanValue();
   }

   public void reset(RobotSide side)
   {
      isRotating.get(side).setFalse();
      hasAdjusted.get(side).setFalse();
   }

   public boolean isRotating(RobotSide side)
   {
      return isRotating.get(side).booleanValue();
   }

   public FramePoint3DReadOnly getDesiredCoP(RobotSide side)
   {
      return desiredCoPs.get(side);
   }

}

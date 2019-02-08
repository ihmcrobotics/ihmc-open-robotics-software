package us.ihmc.commonWalkingControlModules.controlModules.foot.wobble;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootRotationInformation
{
   private final SideDependentList<AtomicBoolean> isRotating = new SideDependentList<>(new AtomicBoolean(), new AtomicBoolean());
   private final SideDependentList<AtomicBoolean> hasAdjusted = new SideDependentList<>(new AtomicBoolean(), new AtomicBoolean());
   private final SideDependentList<FramePoint3DBasics> desiredCoPs = new SideDependentList<FramePoint3DBasics>(new FramePoint3D(), new FramePoint3D());

   public void setRotating(RobotSide side, FramePoint3DReadOnly desiredCop)
   {
      isRotating.get(side).set(true);
      desiredCoPs.get(side).setIncludingFrame(desiredCop);
   }

   public void setAdjusted(RobotSide side)
   {
      hasAdjusted.get(side).set(true);
   }

   public boolean hasAdjusted(RobotSide side)
   {
      return hasAdjusted.get(side).get();
   }

   public void reset(RobotSide side)
   {
      isRotating.get(side).set(false);
      hasAdjusted.get(side).set(false);
   }

   public boolean isRotating(RobotSide side)
   {
      return isRotating.get(side).get();
   }

   public FramePoint3DReadOnly getDesiredCoP(RobotSide side)
   {
      return desiredCoPs.get(side);
   }

}

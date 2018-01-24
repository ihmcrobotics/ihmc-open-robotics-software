package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class ICPPlanToeOffCalculator implements ToeOffCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final String namePrefix = "icpProj";

   private final SideDependentList<List<YoContactPoint>> contactPoints = new SideDependentList<>();

   private final FramePoint3D exitCMP = new FramePoint3D();
   private final FramePoint2D exitCMP2d = new FramePoint2D();

   private final FramePoint2D toeOffContactPoint2d = new FramePoint2D();
   private final LineSegment2D toeOffContactLine2d = new LineSegment2D();

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final YoBoolean hasComputedToeOffContactPoint;
   private final YoBoolean hasComputedToeOffContactLine;

   public ICPPlanToeOffCalculator(SideDependentList<YoPlaneContactState> contactStates, SideDependentList<? extends ContactablePlaneBody> feet,
         YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = feet.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);

         contactPoints.put(robotSide, contactStates.get(robotSide).getContactPoints());
      }

      hasComputedToeOffContactPoint = new YoBoolean(namePrefix + "HasComputedToeOffContactPoint", registry);
      hasComputedToeOffContactLine = new YoBoolean(namePrefix + "HasComputedToeOffContactLine", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public ToeOffEnum getEnum()
   {
      return ToeOffEnum.ICP_PLAN;
   }

   @Override
   public void clear()
   {
      exitCMP2d.setToNaN();
      hasComputedToeOffContactPoint.set(false);
   }

   @Override
   public void setExitCMP(FramePoint3D exitCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);
      this.exitCMP.setIncludingFrame(exitCMP);
      this.exitCMP.changeFrame(soleFrame);
      exitCMP2d.setToZero(soleFrame);
      exitCMP2d.setIncludingFrame(this.exitCMP);
   }

   @Override
   public void computeToeOffContactPoint(FramePoint2D desiredCMP, RobotSide trailingLeg)
   {
      hasComputedToeOffContactPoint.set(true);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2D contactPointToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactPoint.getBooleanValue())
         computeToeOffContactPoint(null, trailingLeg);

      contactPointToPack.set(toeOffContactPoint2d);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2D desiredCMP, RobotSide trailingLeg)
   {
      hasComputedToeOffContactLine.set(true);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2D contactLineToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactLine.getBooleanValue())
         computeToeOffContactLine(null, trailingLeg);

      contactLineToPack.set(toeOffContactLine2d);
   }
}

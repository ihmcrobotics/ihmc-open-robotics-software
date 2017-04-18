package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class ICPPlanToeOffCalculator implements ToeOffCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SideDependentList<List<YoContactPoint>> contactPoints = new SideDependentList<>();

   private final FramePoint exitCMP = new FramePoint();
   private final FramePoint2d toeOffContactPoint2d = new FramePoint2d();
   private final FramePoint2d exitCMP2d = new FramePoint2d();

   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();

   private final BooleanYoVariable hasComputedToeOffContactPoint;

   public ICPPlanToeOffCalculator(SideDependentList<YoPlaneContactState> contactStates, SideDependentList<? extends ContactablePlaneBody> feet,
                       YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ReferenceFrame soleFrame = feet.get(robotSide).getSoleFrame();
         soleFrames.put(robotSide, soleFrame);

         contactPoints.put(robotSide, contactStates.get(robotSide).getContactPoints());
      }

      hasComputedToeOffContactPoint = new BooleanYoVariable("hasComputedToeOffContactPoint", registry);

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      exitCMP2d.setToNaN();
      hasComputedToeOffContactPoint.set(false);
   }

   public void setExitCMP(FramePoint exitCMP, RobotSide trailingLeg)
   {
      ReferenceFrame soleFrame = soleFrames.get(trailingLeg);
      this.exitCMP.setIncludingFrame(exitCMP);
      this.exitCMP.changeFrame(soleFrame);
      exitCMP2d.setToZero(soleFrame);
      exitCMP2d.setByProjectionOntoXYPlaneIncludingFrame(this.exitCMP);
   }

   public void computeToeOffContactPoint(FramePoint2d desiredCMP, RobotSide trailingLeg)
   {
      hasComputedToeOffContactPoint.set(true);
   }

   public void getToeOffContactPoint(FramePoint2d contactPointToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactPoint.getBooleanValue())
         computeToeOffContactPoint(null, trailingLeg);

      contactPointToPack.set(toeOffContactPoint2d);
   }
}

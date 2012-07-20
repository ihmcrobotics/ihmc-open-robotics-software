package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class ConstantCoPInstantaneousCapturePointTrajectory
{
   private final YoVariableRegistry registry;
   private final RobotSide supportSide;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final double gravity;
   private final double deltaT;
   private final SideDependentList<YoFramePoint2d> initialDesiredICP = new SideDependentList<YoFramePoint2d>();
   private final SideDependentList<YoFramePoint2d> finalDesiredICP = new SideDependentList<YoFramePoint2d>();
   private final EnumYoVariable<RobotSide> sideToUseForReferenceFrame;
   private final DoubleYoVariable moveTime;
   private final DoubleYoVariable currentTime;

   public ConstantCoPInstantaneousCapturePointTrajectory(RobotSide supportSide, BipedSupportPolygons bipedSupportPolygons, double gravity, double deltaT, YoVariableRegistry parentRegistry)
   {
      this.supportSide = supportSide;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.gravity = gravity;
      this.deltaT = deltaT;
      this.registry = new YoVariableRegistry(supportSide.getCamelCaseNameForStartOfExpression() + getClass().getSimpleName());

      for (RobotSide robotSide : RobotSide.values())
      {
         ReferenceFrame referenceFrame = bipedSupportPolygons.getFootPolygonInAnkleZUp(robotSide).getReferenceFrame();
         initialDesiredICP.put(robotSide, new YoFramePoint2d(robotSide + "initialDesiredICP", "", referenceFrame, registry));
         finalDesiredICP.put(robotSide, new YoFramePoint2d(robotSide + "finalDesiredICP", "", referenceFrame, registry));

      }

      sideToUseForReferenceFrame = EnumYoVariable.create("sideToUseForReferenceFrame", RobotSide.class, registry);
      moveTime = new DoubleYoVariable("icpTrajectoryMoveTime", registry);
      currentTime = new DoubleYoVariable("icpTrajectoryCurrentTime", registry);

      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, double moveTime, double comHeight)
   {
      sideToUseForReferenceFrame.set(supportSide);
      initialDesiredICP.changeFrame(this.initialDesiredICP.get(supportSide).getReferenceFrame());
      finalDesiredICP.changeFrame(this.finalDesiredICP.get(supportSide).getReferenceFrame());

      this.initialDesiredICP.get(supportSide).set(initialDesiredICP);
      this.finalDesiredICP.get(supportSide).set(finalDesiredICP);
      currentTime.set(0.0);

      // make sure it is feasible by adjusting move time
      FramePoint2d equivalentConstantCoP = EquivalentConstantCoPCalculator.computeEquivalentConstantCoP(initialDesiredICP, finalDesiredICP, moveTime,
                                              comHeight, gravity);
      if (!initialDesiredICP.epsilonEquals(finalDesiredICP, 0.0))
      {
         FrameConvexPolygon2d footPolygonInAnkleZUp = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportSide);

         FrameLine2d line = new FrameLine2d(initialDesiredICP, finalDesiredICP);
         line.changeFrame(footPolygonInAnkleZUp.getReferenceFrame());
         equivalentConstantCoP.changeFrame(footPolygonInAnkleZUp.getReferenceFrame());
         GeometryTools.movePointInsidePolygonAlongLine(equivalentConstantCoP, footPolygonInAnkleZUp, line);
         equivalentConstantCoP.changeFrame(initialDesiredICP.getReferenceFrame());
         moveTime = EquivalentConstantCoPCalculator.computeMoveTime(initialDesiredICP, finalDesiredICP, equivalentConstantCoP, comHeight, gravity);
      }
      this.moveTime.set(moveTime);
   }

   public void pack(FramePoint2d desiredPosition, FrameVector2d desiredVelocity, double comHeight)
   {
      RobotSide sideToUseForReferenceFrame = this.sideToUseForReferenceFrame.getEnumValue();
      double currentTime = isDone() ? moveTime.getDoubleValue() : this.currentTime.getDoubleValue();

      double omega0 = Math.sqrt(gravity / comHeight);
      double expT = Math.exp(omega0 * currentTime);
      double expTf = Math.exp(omega0 * moveTime.getDoubleValue());
      double parameter = (expT - 1.0) / (expTf - 1.0);
      double parameterd = omega0 * expT / (expTf - 1.0);

      FrameVector2d initialToFinal = new FrameVector2d(finalDesiredICP.get(sideToUseForReferenceFrame).getFramePoint2dCopy());
      initialToFinal.sub(initialDesiredICP.get(sideToUseForReferenceFrame).getFramePoint2dCopy());

      FrameVector2d offset = new FrameVector2d(initialToFinal);
      offset.scale(parameter);
      FramePoint2d desiredICPLocal = initialDesiredICP.get(sideToUseForReferenceFrame).getFramePoint2dCopy();
      desiredICPLocal.add(offset);
      desiredICPLocal.changeFrame(desiredPosition.getReferenceFrame());
      desiredPosition.set(desiredICPLocal);

      FrameVector2d desiredICPVelocityLocal = new FrameVector2d(initialToFinal);
      desiredICPVelocityLocal.scale(parameterd);
      desiredICPVelocityLocal.changeFrame(desiredVelocity.getReferenceFrame());
      desiredVelocity.set(desiredICPVelocityLocal);

      this.currentTime.add(deltaT);
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() > moveTime.getDoubleValue();
   }

   public double getMoveTime()
   {
      return moveTime.getDoubleValue();
   }

   public void reset()
   {
      currentTime.set(Double.POSITIVE_INFINITY);
   }

   public void changeSideToUseForReferenceFrame(RobotSide robotSide)
   {
      FramePoint2d initialDesiredICPLocal = initialDesiredICP.get(robotSide.getOppositeSide()).getFramePoint2dCopy();
      initialDesiredICPLocal.changeFrame(initialDesiredICP.get(robotSide).getReferenceFrame());
      initialDesiredICP.get(robotSide).set(initialDesiredICPLocal);

      FramePoint2d finalDesiredICPLocal = finalDesiredICP.get(robotSide.getOppositeSide()).getFramePoint2dCopy();
      finalDesiredICPLocal.changeFrame(finalDesiredICP.get(robotSide).getReferenceFrame());
      finalDesiredICP.get(robotSide).set(finalDesiredICPLocal);

      this.sideToUseForReferenceFrame.set(robotSide);
   }
}

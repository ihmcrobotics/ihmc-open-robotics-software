package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class ConstantCoPInstantaneousCapturePointTrajectory
{
   private final YoVariableRegistry registry;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final double gravity;
   private final double deltaT;
   private final YoFramePoint2d initialDesiredICP;
   private final YoFramePoint2d finalDesiredICP;
   private final DoubleYoVariable moveTime;
   private final DoubleYoVariable currentTime;
   private final double amountToBeInside;

   public ConstantCoPInstantaneousCapturePointTrajectory(BipedSupportPolygons bipedSupportPolygons, double gravity, double deltaT, double amountToBeInside, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.gravity = gravity;
      this.deltaT = deltaT;
      this.registry = new YoVariableRegistry(getClass().getSimpleName());

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      initialDesiredICP =  new YoFramePoint2d("initialDesiredICP", "", referenceFrame, registry);
      finalDesiredICP = new YoFramePoint2d("finalDesiredICP", "", referenceFrame, registry);

      moveTime = new DoubleYoVariable("icpTrajectoryMoveTime", registry);
      currentTime = new DoubleYoVariable("icpTrajectoryCurrentTime", registry);
      this.amountToBeInside = amountToBeInside;

      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, double moveTime, double comHeight)
   {
      initialDesiredICP.changeFrame(this.initialDesiredICP.getReferenceFrame());
      finalDesiredICP.changeFrame(this.finalDesiredICP.getReferenceFrame());

      this.initialDesiredICP.set(initialDesiredICP);
      this.finalDesiredICP.set(finalDesiredICP);
      currentTime.set(0.0);

      // make sure it is feasible by adjusting move time
      FramePoint2d equivalentConstantCoP = EquivalentConstantCoPCalculator.computeEquivalentConstantCoP(initialDesiredICP, finalDesiredICP, moveTime,
                                              comHeight, gravity);
      if (!initialDesiredICP.epsilonEquals(finalDesiredICP, 0.0))
      {
         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp(); // bipedSupportPolygons.getFootPolygonInAnkleZUp(supportSide);

         FrameLine2d line = new FrameLine2d(initialDesiredICP, finalDesiredICP);
         line.changeFrame(supportPolygon.getReferenceFrame());
         equivalentConstantCoP.changeFrame(supportPolygon.getReferenceFrame());
         GeometryTools.movePointInsidePolygonAlongLine(equivalentConstantCoP, supportPolygon, line, amountToBeInside);
         equivalentConstantCoP.changeFrame(initialDesiredICP.getReferenceFrame());
         moveTime = EquivalentConstantCoPCalculator.computeMoveTime(initialDesiredICP, finalDesiredICP, equivalentConstantCoP, comHeight, gravity);
      }
      this.moveTime.set(moveTime);
   }

   public void pack(FramePoint2d desiredPosition, FrameVector2d desiredVelocity, double comHeight)
   {
      double currentTime = isDone() ? moveTime.getDoubleValue() : this.currentTime.getDoubleValue();

      double omega0 = Math.sqrt(gravity / comHeight);
      double expT = Math.exp(omega0 * currentTime);
      double expTf = Math.exp(omega0 * moveTime.getDoubleValue());
      double parameter = (expT - 1.0) / (expTf - 1.0);
      double parameterd = omega0 * expT / (expTf - 1.0);

      FrameVector2d initialToFinal = new FrameVector2d(finalDesiredICP.getFramePoint2dCopy());
      initialToFinal.sub(initialDesiredICP.getFramePoint2dCopy());

      FrameVector2d offset = new FrameVector2d(initialToFinal);
      offset.scale(parameter);
      FramePoint2d desiredICPLocal = initialDesiredICP.getFramePoint2dCopy();
      desiredICPLocal.add(offset);
      desiredICPLocal.changeFrame(desiredPosition.getReferenceFrame());
      desiredPosition.set(desiredICPLocal);

      if (isDone())
      {
         desiredVelocity.set(0.0, 0.0);
      }
      else
      {
         FrameVector2d desiredICPVelocityLocal = new FrameVector2d(initialToFinal);
         desiredICPVelocityLocal.scale(parameterd);
         desiredICPVelocityLocal.changeFrame(desiredVelocity.getReferenceFrame());
         desiredVelocity.set(desiredICPVelocityLocal);
      }

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
}

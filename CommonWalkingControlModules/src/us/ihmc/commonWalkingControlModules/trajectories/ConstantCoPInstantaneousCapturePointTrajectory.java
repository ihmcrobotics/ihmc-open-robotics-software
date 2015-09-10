package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.calculators.EquivalentConstantCoPCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;


public class ConstantCoPInstantaneousCapturePointTrajectory implements InstantaneousCapturePointTrajectory
{
   private final YoVariableRegistry registry;
   private final BipedSupportPolygons bipedSupportPolygons;
   private final YoFramePoint2d initialDesiredICP;
   private final YoFramePoint2d finalDesiredICP;
   
   private final DoubleYoVariable startTime;
   
   private final DoubleYoVariable moveTime;
   private final DoubleYoVariable currentTime;

   public ConstantCoPInstantaneousCapturePointTrajectory(String namePrefix, BipedSupportPolygons bipedSupportPolygons, YoVariableRegistry parentRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      initialDesiredICP = new YoFramePoint2d(namePrefix + "InitialDesiredICP", "", referenceFrame, registry);
      finalDesiredICP = new YoFramePoint2d(namePrefix + "FinalDesiredICP", "", referenceFrame, registry);

      startTime = new DoubleYoVariable(namePrefix + "ICPTrajectoryStartTime", registry);

      moveTime = new DoubleYoVariable(namePrefix + "ICPTrajectoryMoveTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "ICPTrajectoryCurrentTime", registry);

      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize(FramePoint2d initialDesiredICP, FramePoint2d finalDesiredICP, double moveTime, double omega0, double amountToBeInside, RobotSide supportSide, double clockTime)
   {
      initialDesiredICP.changeFrame(this.initialDesiredICP.getReferenceFrame());
      finalDesiredICP.changeFrame(this.finalDesiredICP.getReferenceFrame());

      this.initialDesiredICP.set(initialDesiredICP);
      this.finalDesiredICP.set(finalDesiredICP);
      
      startTime.set(clockTime);
      currentTime.set(0.0);

      double epsilon = 1e-12;
      if (!initialDesiredICP.epsilonEquals(finalDesiredICP, epsilon))
      {
         // make sure it is feasible by adjusting move time
         FramePoint2d equivalentConstantCoP = EquivalentConstantCoPCalculator.computeEquivalentConstantCoP(initialDesiredICP, finalDesiredICP, moveTime,
                                                 omega0);

         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();    // bipedSupportPolygons.getFootPolygonInAnkleZUp(supportSide);

         FrameVector2d vector = new FrameVector2d(initialDesiredICP, finalDesiredICP);
         vector.changeFrame(supportPolygon.getReferenceFrame());
         equivalentConstantCoP.changeFrame(supportPolygon.getReferenceFrame());
         GeometryTools.movePointInsidePolygonAlongVector(equivalentConstantCoP, vector, supportPolygon, amountToBeInside);

         equivalentConstantCoP.changeFrame(initialDesiredICP.getReferenceFrame());
         moveTime = EquivalentConstantCoPCalculator.computeMoveTime(initialDesiredICP, finalDesiredICP, equivalentConstantCoP, omega0);
      }

      if (Double.isInfinite(moveTime))
         throw new RuntimeException();
      if (moveTime < 0.0)
         throw new MoveTimeNegativeException(moveTime);

      this.moveTime.set(moveTime);
   }
   
   public double getStartTime()
   {
      return startTime.getDoubleValue();
   }

   public void getCurrentDesiredICPPositionAndVelocity(FramePoint2d desiredPosition, FrameVector2d desiredVelocity, double omega0, double clockTime)
   {
      double currentTime = clockTime - this.startTime.getDoubleValue();
      this.currentTime.set(currentTime);
      
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

      FrameVector2d desiredICPVelocityLocal = new FrameVector2d(initialToFinal);
      desiredICPVelocityLocal.scale(parameterd);
      desiredICPVelocityLocal.changeFrame(desiredVelocity.getReferenceFrame());
      desiredVelocity.set(desiredICPVelocityLocal);
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

   public class MoveTimeNegativeException extends RuntimeException
   {
      private static final long serialVersionUID = 356745581088808113L;
      public MoveTimeNegativeException(double moveTime)
      {
         super("Move time negative. moveTime = " + moveTime);
      }
   }


   public double getEstimatedTimeRemainingForState(double time)
   {
      return currentTime.getDoubleValue() - moveTime.getDoubleValue();
   }
}

package us.ihmc.robotics.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.commons.MathTools;

public class WaypointMotionGenerator
{
   private ListOfPointsTrajectory listOfPointsTrajectory;
   private double pathLength;
   private double stepSizeforNumericalCalculation = 0.001;    // 0.001; //0.00001;
   private double moveDuration;

   private MinimumJerkTrajectory minimumJerkTrajectory = new MinimumJerkTrajectory();

   public WaypointMotionGenerator()
   {
   }


   public WaypointMotionGenerator(ListOfPointsTrajectory listOfPointsTrajectory, double moveDuration)
   {
      this(listOfPointsTrajectory, moveDuration, 0.0, 0.0, 0.0, 0.0);
   }

   public WaypointMotionGenerator(ListOfPointsTrajectory listOfPointsTrajectory, double moveDuration, double initialVelocity, double initialAcceleration,
                                  double finalVelocity, double finalAcceleration)
   {
      initialize(listOfPointsTrajectory, moveDuration, initialVelocity, initialAcceleration, finalVelocity, finalAcceleration);
   }


   public void initialize(ListOfPointsTrajectory listOfPointsTrajectory, double moveDuration)
   {
      initialize(listOfPointsTrajectory, moveDuration, 0.0, 0.0, 0.0, 0.0);
   }

   public void initialize(ListOfPointsTrajectory listOfPointsTrajectory, double moveDuration, double initialVelocity, double initialAcceleration,
                          double finalVelocity, double finalAcceleration)
   {
      this.listOfPointsTrajectory = listOfPointsTrajectory;
      pathLength = listOfPointsTrajectory.getPathLength();
      this.moveDuration = moveDuration;

      double initialPosition = 0.0;

      double finalPosition = pathLength;

      minimumJerkTrajectory.setMoveParameters(initialPosition, initialVelocity, initialAcceleration, finalPosition, finalVelocity, finalAcceleration,
              moveDuration);
   }


   public FramePoint3D getCurrentDesiredPoint(double timeInMove)
   {
      minimumJerkTrajectory.computeTrajectory(timeInMove);

      double distanceAlongPath = minimumJerkTrajectory.getPosition();

      return listOfPointsTrajectory.getPointOnPathDistanceFromStart(distanceAlongPath);
   }

   // TODO: Find more elegant way to handle edge cases.
   public FrameVector3D getCurrentDesiredVelocity(double timeInMove)
   {
      FramePoint3D Xfh = getCurrentDesiredPoint(timeInMove + stepSizeforNumericalCalculation);
      FramePoint3D Xf2h = getCurrentDesiredPoint(timeInMove + 2.0 * stepSizeforNumericalCalculation);
      FramePoint3D Xrh = getCurrentDesiredPoint(timeInMove - stepSizeforNumericalCalculation);
      FramePoint3D Xr2h = getCurrentDesiredPoint(timeInMove - 2.0 * stepSizeforNumericalCalculation);

      FrameVector3D ret = new FrameVector3D(Xfh.getReferenceFrame());
      ret.setX((-Xf2h.getX() + 8.0 * Xfh.getX() - 8.0 * Xrh.getX() + Xr2h.getX()) / (12.0 * stepSizeforNumericalCalculation));
      ret.setY((-Xf2h.getY() + 8.0 * Xfh.getY() - 8.0 * Xrh.getY() + Xr2h.getY()) / (12.0 * stepSizeforNumericalCalculation));
      ret.setZ((-Xf2h.getZ() + 8.0 * Xfh.getZ() - 8.0 * Xrh.getZ() + Xr2h.getZ()) / (12.0 * stepSizeforNumericalCalculation));

      // Edge cases result in half the expected value, so multiply by 2
      if ((timeInMove <= stepSizeforNumericalCalculation) || (this.moveDuration - timeInMove <= stepSizeforNumericalCalculation))
      {
         ret.setX(ret.getX() * 2);
         ret.setY(ret.getY() * 2);
         ret.setZ(ret.getZ() * 2);
      }

      return ret;
   }

   // TODO: This does not return the exact acceleration at the start and end of the move
   public FrameVector3D getCurrentDesiredAcceleration(double timeInMove)
   {
      FramePoint3D Xfh = getCurrentDesiredPoint(timeInMove + stepSizeforNumericalCalculation);
      FramePoint3D X = getCurrentDesiredPoint(timeInMove);
      FramePoint3D Xrh = getCurrentDesiredPoint(timeInMove - stepSizeforNumericalCalculation);

      FrameVector3D ret = new FrameVector3D(X.getReferenceFrame());
      ret.setX((Xfh.getX() - 2.0 * X.getX() + Xrh.getX()) / (MathTools.square(stepSizeforNumericalCalculation)));
      ret.setY((Xfh.getY() - 2.0 * X.getY() + Xrh.getY()) / (MathTools.square(stepSizeforNumericalCalculation)));
      ret.setZ((Xfh.getZ() - 2.0 * X.getZ() + Xrh.getZ()) / (MathTools.square(stepSizeforNumericalCalculation)));

      return ret;
   }


   public void setStepSizeforNumericalCalculation(double stepSize)
   {
      stepSizeforNumericalCalculation = stepSize;
   }
}

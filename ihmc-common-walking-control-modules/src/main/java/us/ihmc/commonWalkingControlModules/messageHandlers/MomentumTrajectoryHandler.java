package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.MomentumTrajectoryCommand;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MomentumTrajectoryHandler
{
   private final YoDouble yoTime;
   private final RecyclingArrayList<SimpleEuclideanTrajectoryPoint> angularMomentumTrajectoryPoints = new RecyclingArrayList<>(10,
                                                                                                                               SimpleEuclideanTrajectoryPoint.class);

   private final YoPolynomial polynomial = new YoPolynomial("CubicPolynomial", 4, new YoVariableRegistry("Temp"));
   private final Point3D tempPosition = new Point3D();
   private final Vector3D tempVelocity = new Vector3D();

   private final RuntimeException table = new RuntimeException("This can not be happening.");

   public MomentumTrajectoryHandler(YoDouble yoTime)
   {
      this.yoTime = yoTime;
      angularMomentumTrajectoryPoints.clear();
   }

   public void handleMomentumTrajectory(MomentumTrajectoryCommand command)
   {
      clearPoints();

      switch (command.getAngularMomentumTrajectory().getExecutionMode())
      {
      case OVERRIDE:
         command.getAngularMomentumTrajectory().addTimeOffset(yoTime.getDoubleValue());
         angularMomentumTrajectoryPoints.clear();
         break;
      case QUEUE:
         if (angularMomentumTrajectoryPoints.isEmpty())
         {
            PrintTools.warn("Can not queue without points");
            return;
         }
         if (command.getAngularMomentumTrajectory().getTrajectoryPoint(0).getTime() <= 0.0)
         {
            PrintTools.warn("Can not queue trajectory with initial time 0.0");
            return;
         }
         double lastTime = angularMomentumTrajectoryPoints.getLast().getTime();
         command.getAngularMomentumTrajectory().addTimeOffset(lastTime);
         break;
      default:
         throw table;
      }

      for (int idx = 0; idx < command.getAngularMomentumTrajectory().getNumberOfTrajectoryPoints(); idx++)
      {
         angularMomentumTrajectoryPoints.add().set(command.getAngularMomentumTrajectory().getTrajectoryPoint(idx));
      }
   }

   private void clearPoints()
   {
      double currentTime = yoTime.getDoubleValue();
      while (angularMomentumTrajectoryPoints.size() > 1 && angularMomentumTrajectoryPoints.get(0).getTime() < currentTime)
      {
         angularMomentumTrajectoryPoints.remove(0);
      }
   }

   public void getAngularMomentumTrajectory(double startTime, double endTime, int numberOfPoints,
                                            RecyclingArrayList<SimpleEuclideanTrajectoryPoint> trajectoryToPack)
   {
      trajectoryToPack.clear();
      if (startTime <= angularMomentumTrajectoryPoints.get(0).getTime())
      {
         return;
      }
      if (endTime > angularMomentumTrajectoryPoints.getLast().getTime())
      {
         return;
      }
      if (numberOfPoints < 2)
      {
         return;
      }

      for (int idx = 0; idx < numberOfPoints; idx++)
      {
         double time = startTime + (endTime - startTime) * idx / (numberOfPoints - 1);
         SimpleEuclideanTrajectoryPoint trajectoryPoint = trajectoryToPack.add();
         packPointAtTime(time, trajectoryPoint);
         trajectoryPoint.setTime(trajectoryPoint.getTime() - startTime);
      }
   }

   private void packPointAtTime(double time, SimpleEuclideanTrajectoryPoint trajectoryPoint)
   {
      trajectoryPoint.setTime(time);

      int endIndex = 0;
      while (angularMomentumTrajectoryPoints.get(endIndex).getTime() < time)
      {
         endIndex++;
      }

      SimpleEuclideanTrajectoryPoint startPoint = angularMomentumTrajectoryPoints.get(endIndex - 1);
      SimpleEuclideanTrajectoryPoint endPoint = angularMomentumTrajectoryPoints.get(endIndex);

      double t0 = startPoint.getTime();
      double t1 = endPoint.getTime();

      for (int i = 0; i < 3; i++)
      {
         double p0 = startPoint.getEuclideanWaypoint().getPosition().getElement(i);
         double v0 = startPoint.getEuclideanWaypoint().getLinearVelocity().getElement(i);
         double p1 = endPoint.getEuclideanWaypoint().getPosition().getElement(i);
         double v1 = endPoint.getEuclideanWaypoint().getLinearVelocity().getElement(i);

         polynomial.setCubic(t0, t1, p0, v0, p1, v1);
         polynomial.compute(time);

         tempPosition.setElement(i, polynomial.getPosition());
         tempVelocity.setElement(i, polynomial.getVelocity());
      }

      trajectoryPoint.setPosition(tempPosition);
      trajectoryPoint.setLinearVelocity(tempVelocity);
   }
}

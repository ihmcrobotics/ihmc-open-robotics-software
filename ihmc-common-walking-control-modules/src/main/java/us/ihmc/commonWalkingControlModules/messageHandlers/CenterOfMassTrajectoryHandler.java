package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CenterOfMassTrajectoryCommand;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CenterOfMassTrajectoryHandler
{
   private final YoDouble yoTime;
   private final RecyclingArrayList<SimpleEuclideanTrajectoryPoint> comTrajectoryPoints = new RecyclingArrayList<>(10, SimpleEuclideanTrajectoryPoint.class);

   private final YoPolynomial polynomial = new YoPolynomial("CubicPolynomial", 4, new YoVariableRegistry("Temp"));

   private final Point3D comPosition = new Point3D();
   private final Vector3D comVelocity = new Vector3D();
   private final Vector3D comAcceleration = new Vector3D();

   private final Point3D icpPosition = new Point3D();
   private final Vector3D icpVelocity = new Vector3D();

   public CenterOfMassTrajectoryHandler(YoDouble yoTime)
   {
      this.yoTime = yoTime;
      comTrajectoryPoints.clear();
   }

   /**
    * Handles the provided command. This method will extract the desired center of mass trajectory from the command and append
    * it to the internal trajectory maintained in this class.
    *
    * @param command
    */
   public void handleComTrajectory(CenterOfMassTrajectoryCommand command)
   {
      clearPoints();

      switch (command.getEuclideanTrajectory().getExecutionMode())
      {
      case OVERRIDE:
         command.getEuclideanTrajectory().addTimeOffset(yoTime.getDoubleValue());
         comTrajectoryPoints.clear();
         break;
      case QUEUE:
         if (comTrajectoryPoints.isEmpty())
         {
            PrintTools.warn("Can not queue without points");
            return;
         }
         if (command.getEuclideanTrajectory().getTrajectoryPoint(0).getTime() <= 0.0)
         {
            PrintTools.warn("Can not queue trajectory with initial time 0.0");
            return;
         }
         double lastTime = comTrajectoryPoints.getLast().getTime();
         command.getEuclideanTrajectory().addTimeOffset(lastTime);
         break;
      default:
         throw new RuntimeException("Unhadled execution mode.");
      }

      for (int idx = 0; idx < command.getEuclideanTrajectory().getNumberOfTrajectoryPoints(); idx++)
      {
         comTrajectoryPoints.add().set(command.getEuclideanTrajectory().getTrajectoryPoint(idx));
      }
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the current controller time. If the handler does not have
    * a valid trajectory for this time the method will return false and set the desireds to be NaN.
    *
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packCurrentDesiredICP(double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack)
   {
      return packCurrentDesiredICP(omega0, desiredICPPositionToPack, desiredICPVelocityToPack, null);
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the current controller time. If the handler does not have
    * a valid trajectory for this time the method will return false and set the desireds to be NaN. This method also packs the
    * current desired center of mass position for visualization.
    *
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @param comPositionToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packCurrentDesiredICP(double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack, FramePoint3D comPositionToPack)
   {
      return packDesiredICPAtTime(yoTime.getDoubleValue(), omega0, desiredICPPositionToPack, desiredICPVelocityToPack, comPositionToPack);
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the provided controller time. If the handler does not have
    * a valid trajectory for this time the method will return false and set the desireds to be NaN.
    *
    * @param controllerTime
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packDesiredICPAtTime(double controllerTime, double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack)
   {
      return packDesiredICPAtTime(controllerTime, omega0, desiredICPPositionToPack, desiredICPVelocityToPack, null);
   }

   /**
    * Computed and packs the ICP desireds (position and velocity) at the provided controller time. If the handler does not have
    * a valid trajectory for this time the method will return false and set the desireds to be NaN.  This method also packs the
    * current desired center of mass position for visualization.
    *
    * @param controllerTime
    * @param omega0
    * @param desiredICPPositionToPack (modified)
    * @param desiredICPVelocityToPack (modified)
    * @param comPositionToPack (modified)
    * @return whether a valid trajectory point for this time was available
    */
   public boolean packDesiredICPAtTime(double controllerTime, double omega0, FramePoint3D desiredICPPositionToPack, FrameVector3D desiredICPVelocityToPack,
                                       FramePoint3D comPositionToPack)
   {
      if (!isWithinInterval(controllerTime))
      {
         desiredICPPositionToPack.setToNaN(ReferenceFrame.getWorldFrame());
         desiredICPVelocityToPack.setToNaN(ReferenceFrame.getWorldFrame());
         if (comPositionToPack != null)
         {
            comPositionToPack.setToNaN(ReferenceFrame.getWorldFrame());
         }
         return false;
      }

      packDesiredsAtTime(controllerTime, comPosition, comVelocity, comAcceleration);
      icpPosition.scaleAdd(1.0 / omega0, comVelocity, comPosition);
      icpVelocity.scaleAdd(1.0 / omega0, comAcceleration, comVelocity);

      desiredICPPositionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), icpPosition);
      desiredICPVelocityToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), icpVelocity);
      desiredICPPositionToPack.add(offset);

      if (comPositionToPack != null)
      {
         comPositionToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), comPosition);
         comPositionToPack.add(offset);
      }
      return true;
   }

   /**
    * Returns whether or not the supplied time is within the trajectory time interval specified in this handler. The interval
    * is closed (contains the end points).
    *
    * @param controllerTime
    * @return whether the provided time is contained in the closed trajectory time interval
    */
   public boolean isWithinInterval(double controllerTime)
   {
      if (comTrajectoryPoints.size() < 2)
      {
         return false;
      }
      if (controllerTime < comTrajectoryPoints.get(0).getTime())
      {
         return false;
      }
      if (controllerTime > comTrajectoryPoints.getLast().getTime())
      {
         return false;
      }
      return true;
   }

   private void clearPoints()
   {
      double currentTime = yoTime.getDoubleValue();
      while (comTrajectoryPoints.size() > 1 && comTrajectoryPoints.get(0).getTime() < currentTime)
      {
         comTrajectoryPoints.remove(0);
      }
   }

   private void packDesiredsAtTime(double time, Point3D comPosition, Vector3D comVelocity, Vector3D comAcceleration)
   {
      int endIndex = 0;
      while (comTrajectoryPoints.get(endIndex).getTime() < time)
      {
         endIndex++;
      }

      if (endIndex == 0)
      {
         endIndex++;
      }

      SimpleEuclideanTrajectoryPoint startPoint = comTrajectoryPoints.get(endIndex - 1);
      SimpleEuclideanTrajectoryPoint endPoint = comTrajectoryPoints.get(endIndex);

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

         comPosition.setElement(i, polynomial.getPosition());
         comVelocity.setElement(i, polynomial.getVelocity());
         comAcceleration.setElement(i, polynomial.getAcceleration());
      }
   }

   private final FrameVector3D offset = new FrameVector3D();
   public void setPositionOffset(FrameVector3D offset)
   {
      this.offset.setIncludingFrame(offset);
   }

}

package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.MultiContactBalanceStatus;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.util.List;

/**
 * API for a module external to the controller to inform balance status
 */
public class MultiContactBalanceStatusCommand implements Command<MultiContactBalanceStatusCommand, MultiContactBalanceStatus>
{
   private long sequenceId;
   private final Point2D capturePoint2D = new Point2D();
   private final Point3D centerOfMass3D = new Point3D();
   private final RecyclingArrayList<Point3D> contactPointsInWorld = new RecyclingArrayList<>(10, Point3D::new);
   private final RecyclingArrayList<Vector3D> surfaceNormalsInWorld = new RecyclingArrayList<>(10, Vector3D::new);
   private final TIntArrayList supportRigidBodies = new TIntArrayList(10);

   @Override
   public void clear()
   {
      sequenceId = 0;
      capturePoint2D.setToZero();
      centerOfMass3D.setToZero();
      contactPointsInWorld.clear();
      surfaceNormalsInWorld.clear();
      supportRigidBodies.reset();
   }

   @Override
   public void setFromMessage(MultiContactBalanceStatus message)
   {
      sequenceId = message.getSequenceId();
      capturePoint2D.set(message.getCapturePoint2d());
      centerOfMass3D.set(message.getCenterOfMass3d());

      contactPointsInWorld.clear();
      for (int i = 0; i < message.getContactPointsInWorld().size(); i++)
      {
         contactPointsInWorld.add().set(message.getContactPointsInWorld().get(i));
      }

      surfaceNormalsInWorld.clear();
      for (int i = 0; i < message.getSurfaceNormalsInWorld().size(); i++)
      {
         surfaceNormalsInWorld.add().set(message.getSurfaceNormalsInWorld().get(i));
      }

      supportRigidBodies.reset();
      for (int i = 0; i < message.getSupportRigidBodyIds().size(); i++)
      {
         supportRigidBodies.add(message.getSupportRigidBodyIds().get(i));
      }
   }

   @Override
   public Class<MultiContactBalanceStatus> getMessageClass()
   {
      return MultiContactBalanceStatus.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public Point2D getCapturePoint2D()
   {
      return capturePoint2D;
   }

   public Point3D getCenterOfMass3D()
   {
      return centerOfMass3D;
   }

   public List<Point3D> getContactPointsInWorld()
   {
      return contactPointsInWorld;
   }

   public List<Vector3D> getSurfaceNormalsInWorld()
   {
      return surfaceNormalsInWorld;
   }

   public TIntArrayList getSupportRigidBodiesHashCodes()
   {
      return supportRigidBodies;
   }

   @Override
   public void set(MultiContactBalanceStatusCommand other)
   {
      sequenceId = other.sequenceId;
      capturePoint2D.set(other.capturePoint2D);
      centerOfMass3D.set(other.centerOfMass3D);

      contactPointsInWorld.clear();
      for (int i = 0; i < other.getContactPointsInWorld().size(); i++)
      {
         contactPointsInWorld.add().set(other.getContactPointsInWorld().get(i));
      }

      surfaceNormalsInWorld.clear();
      for (int i = 0; i < other.getSurfaceNormalsInWorld().size(); i++)
      {
         surfaceNormalsInWorld.add().set(other.getSurfaceNormalsInWorld().get(i));
      }

      supportRigidBodies.reset();
      for (int i = 0; i < other.getSupportRigidBodiesHashCodes().size(); i++)
      {
         supportRigidBodies.add(other.getSupportRigidBodiesHashCodes().get(i));
      }
   }
}

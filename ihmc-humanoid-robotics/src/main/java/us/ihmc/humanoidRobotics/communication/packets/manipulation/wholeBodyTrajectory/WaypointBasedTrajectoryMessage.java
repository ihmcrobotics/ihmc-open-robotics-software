package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class WaypointBasedTrajectoryMessage extends Packet<WaypointBasedTrajectoryMessage>
{
   /**
    * This is the unique hash code of the end-effector to be solved for. It used on the solver side
    * to retrieve the desired end-effector to be controlled.
    */
   public long endEffectorNameBasedHashCode;
   public TDoubleArrayList waypointTimes = new TDoubleArrayList();
   public RecyclingArrayListPubSub<Pose3D> waypoints = new RecyclingArrayListPubSub<>(Pose3D.class, Pose3D::new, 50);
   public SelectionMatrix3DMessage angularSelectionMatrix = new SelectionMatrix3DMessage();
   public SelectionMatrix3DMessage linearSelectionMatrix = new SelectionMatrix3DMessage();
   /**
    * This is the position of the control frame's origin expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Point3D32 controlFramePositionInEndEffector = new Point3D32();
   /**
    * This is the orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Quaternion32 controlFrameOrientationInEndEffector = new Quaternion32();

   public double weight = Double.NaN;

   public WaypointBasedTrajectoryMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(WaypointBasedTrajectoryMessage other)
   {
      endEffectorNameBasedHashCode = other.endEffectorNameBasedHashCode;
      MessageTools.copyData(other.waypointTimes, waypointTimes);
      MessageTools.copyData(other.waypoints, waypoints);
      angularSelectionMatrix.set(other.angularSelectionMatrix);
      linearSelectionMatrix.set(other.linearSelectionMatrix);
      controlFramePositionInEndEffector.set(other.controlFramePositionInEndEffector);
      controlFrameOrientationInEndEffector.set(other.controlFrameOrientationInEndEffector);
      weight = other.weight;
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   /**
    * Specifies the position of the control frame's origin. The given point is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePositionInEndEffector(Tuple3DReadOnly controlFramePosition)
   {
      controlFramePositionInEndEffector.set(controlFramePosition);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientationInEndEffector(QuaternionReadOnly controlFrameOrientation)
   {
      controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   public long getEndEffectorNameBasedHashCode()
   {
      return endEffectorNameBasedHashCode;
   }

   public TDoubleArrayList getWaypointTimes()
   {
      return waypointTimes;
   }

   public RecyclingArrayListPubSub<Pose3D> getWaypoints()
   {
      return waypoints;
   }

   public SelectionMatrix3DMessage getAngularSelectionMatrix()
   {
      return angularSelectionMatrix;
   }

   public SelectionMatrix3DMessage getLinearSelectionMatrix()
   {
      return linearSelectionMatrix;
   }

   @Override
   public boolean epsilonEquals(WaypointBasedTrajectoryMessage other, double epsilon)
   {
      if (waypoints.size() != other.waypoints.size())
         return false;

      if (!MessageTools.epsilonEquals(waypointTimes, other.waypointTimes, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(waypoints, other.waypoints, epsilon))
         return false;
      if (!angularSelectionMatrix.epsilonEquals(other.angularSelectionMatrix, epsilon))
         return false;
      if (!linearSelectionMatrix.epsilonEquals(other.linearSelectionMatrix, epsilon))
         return false;

      if (weight != other.weight)
         return false;

      return true;
   }
}

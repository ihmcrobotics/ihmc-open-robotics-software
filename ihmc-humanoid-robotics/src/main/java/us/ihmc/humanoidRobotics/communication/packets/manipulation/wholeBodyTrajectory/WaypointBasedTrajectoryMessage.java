package us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.SelectionMatrix3DMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.tools.ArrayTools;

public class WaypointBasedTrajectoryMessage extends Packet<WaypointBasedTrajectoryMessage>
{
   /**
    * This is the unique hash code of the end-effector to be solved for. It used on the solver side
    * to retrieve the desired end-effector to be controlled.
    */
   public long endEffectorNameBasedHashCode;
   public double[] waypointTimes;
   public Pose3D[] waypoints;
   public SelectionMatrix3DMessage angularSelectionMatrix = new SelectionMatrix3DMessage();
   public SelectionMatrix3DMessage linearSelectionMatrix = new SelectionMatrix3DMessage();
   /**
    * This is the position of the control frame's origin expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Point3D32 controlFramePositionInEndEffector;
   /**
    * This is the orientation of the control frame expressed in
    * {@code endEffector.getBodyFixedFrame()}. By default the control frame is coincident to
    * {@code endEffector.getBodyFixedFrame()}. The control frame is assumed to be attached to the
    * end-effector.
    */
   public Quaternion32 controlFrameOrientationInEndEffector;
   
   public double weight = Double.NaN;

   public WaypointBasedTrajectoryMessage()
   {
      // empty constructor for deserialization
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public WaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints)
   {
      this(endEffector, waypointTimes, waypoints, null);
   }

   public WaypointBasedTrajectoryMessage(RigidBody endEffector, double[] waypointTimes, Pose3D[] waypoints, SelectionMatrix6D selectionMatrix)
   {
      endEffectorNameBasedHashCode = endEffector.getNameBasedHashCode();
      setWaypoints(waypointTimes, waypoints);
      if (selectionMatrix != null)
      {
         angularSelectionMatrix.set(selectionMatrix.getAngularPart());
         linearSelectionMatrix.set(selectionMatrix.getLinearPart());
      }
      setUniqueId(Packet.VALID_MESSAGE_DEFAULT_ID);
   }

   public void setWaypoints(double[] waypointTimes, Pose3D[] waypoints)
   {
      if (waypointTimes.length != waypoints.length)
         throw new RuntimeException("Inconsistent array lengths.");

      this.waypointTimes = waypointTimes;
      this.waypoints = waypoints;
   }
   
   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   /**
    * Enables the control of all the degrees of freedom of the end-effector.
    */
   public void setSelectionMatrixToIdentity()
   {
      angularSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix = new SelectionMatrix3DMessage();
   }

   /**
    * Enables the control for the translational degrees of freedom of the end-effector and disable
    * the rotational part.
    */
   public void setSelectionMatrixForLinearControl()
   {
      angularSelectionMatrix = new SelectionMatrix3DMessage();
      angularSelectionMatrix.setAxisSelection(false, false, false);
      linearSelectionMatrix = new SelectionMatrix3DMessage();
   }

   /**
    * Enables the control for the rotational degrees of freedom of the end-effector and disable the
    * translational part.
    */
   public void setSelectionMatrixForAngularControl()
   {
      angularSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix = new SelectionMatrix3DMessage();
      linearSelectionMatrix.setAxisSelection(false, false, false);
   }

   /**
    * Sets the selection matrix to use for executing this message.
    * <p>
    * The selection matrix is used to determinate which degree of freedom of the end-effector should
    * be controlled. When it is NOT provided, the controller will assume that all the degrees of
    * freedom of the end-effector should be controlled.
    * </p>
    * <p>
    * The selection frames coming along with the given selection matrix are used to determine to
    * what reference frame the selected axes are referring to. For instance, if only the hand height
    * in world should be controlled on the linear z component of the selection matrix should be
    * selected and the reference frame should be world frame. When no reference frame is provided
    * with the selection matrix, it will be used as it is in the control frame, i.e. the body-fixed
    * frame if not defined otherwise.
    * </p>
    * 
    * @param selectionMatrix the selection matrix to use when executing this trajectory message. Not
    *           modified.
    */
   public void setSelectionMatrix(SelectionMatrix6D selectionMatrix6D)
   {
      if (angularSelectionMatrix == null)
         angularSelectionMatrix = new SelectionMatrix3DMessage(selectionMatrix6D.getAngularPart());
      else
         angularSelectionMatrix.set(selectionMatrix6D.getAngularPart());

      if (linearSelectionMatrix == null)
         linearSelectionMatrix = new SelectionMatrix3DMessage(selectionMatrix6D.getLinearPart());
      else
         linearSelectionMatrix.set(selectionMatrix6D.getLinearPart());
   }

   /**
    * Resets the control frame so it is coincident with {@code endEffector.getBodyFixedFrame()}.
    */
   public void resetControlFrame()
   {
      if (controlFramePositionInEndEffector != null)
         controlFramePositionInEndEffector.setToZero();
      if (controlFrameOrientationInEndEffector != null)
         controlFrameOrientationInEndEffector.setToZero();
   }

   /**
    * Specifies the position of the control frame's origin. The given point is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePosition(Tuple3DReadOnly controlFramePosition)
   {
      if (controlFramePositionInEndEffector == null)
         controlFramePositionInEndEffector = new Point3D32(controlFramePosition);
      else
         controlFramePositionInEndEffector.set(controlFramePosition);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientation(QuaternionReadOnly controlFrameOrientation)
   {
      if (controlFrameOrientationInEndEffector == null)
         controlFrameOrientationInEndEffector = new Quaternion32(controlFrameOrientation);
      else
         controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   /**
    * Specifies the orientation of the control frame. The given quaternion is assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFrameOrientation(RotationMatrixReadOnly controlFrameOrientation)
   {
      if (controlFrameOrientationInEndEffector == null)
         controlFrameOrientationInEndEffector = new Quaternion32(controlFrameOrientation);
      else
         controlFrameOrientationInEndEffector.set(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given point and quaternion are assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Tuple3DReadOnly controlFramePosition, QuaternionReadOnly controlFrameOrientation)
   {
      setControlFramePosition(controlFramePosition);
      setControlFrameOrientation(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given point and quaternion are assumed to be
    * expressed in {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePosition the position of the control frame's origin expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    * @param controlFrameOrientation the orientation of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Tuple3DReadOnly controlFramePosition, RotationMatrixReadOnly controlFrameOrientation)
   {
      setControlFramePosition(controlFramePosition);
      setControlFrameOrientation(controlFrameOrientation);
   }

   /**
    * Specifies the pose of the control frame. The given pose is assumed to be expressed in
    * {@code endEffector.getBodyFixedFrame()}.
    * 
    * @param controlFramePose the pose of the control frame expressed in
    *           {@code endEffector.getBodyFixedFrame()}. Not modified.
    */
   public void setControlFramePose(Pose3D controlFramePose)
   {
      setControlFramePosition(controlFramePose.getPosition());
      setControlFrameOrientation(controlFramePose.getOrientation());
   }

   public long getEndEffectorNameBasedHashCode()
   {
      return endEffectorNameBasedHashCode;
   }

   public double getWaypointTime(int i)
   {
      return waypointTimes[i];
   }

   public double getLastWaypointTime()
   {
      return waypointTimes[getNumberOfWaypoints() - 1];
   }

   public double[] getWaypointTimes()
   {
      return waypointTimes;
   }

   public Pose3D getWaypoint(int i)
   {
      return waypoints[i];
   }

   public Pose3D[] getWaypoints()
   {
      return waypoints;
   }

   public Pose3D getPose(double time)
   {
      if (time <= 0.0)
         return waypoints[0];

      else if (time >= waypointTimes[waypointTimes.length - 1])
         return waypoints[waypoints.length - 1];

      else
      {
         double timeGap = 0.0;

         int indexOfFrame = 0;
         int numberOfTrajectoryTimes = waypointTimes.length;

         for (int i = 0; i < numberOfTrajectoryTimes; i++)
         {
            timeGap = time - waypointTimes[i];
            if (timeGap < 0)
            {
               indexOfFrame = i;
               break;
            }
         }

         Pose3D poseOne = waypoints[indexOfFrame - 1];
         Pose3D poseTwo = waypoints[indexOfFrame];

         double timeOne = waypointTimes[indexOfFrame - 1];
         double timeTwo = waypointTimes[indexOfFrame];

         double alpha = (time - timeOne) / (timeTwo - timeOne);

         Pose3D ret = new Pose3D();
         ret.interpolate(poseOne, poseTwo, alpha);

         return ret;
      }
   }

   public void getSelectionMatrix(SelectionMatrix6D selectionMatrixToPack)
   {
      selectionMatrixToPack.resetSelection();
      if (angularSelectionMatrix != null)
         angularSelectionMatrix.getSelectionMatrix(selectionMatrixToPack.getAngularPart());
      if (linearSelectionMatrix != null)
         linearSelectionMatrix.getSelectionMatrix(selectionMatrixToPack.getLinearPart());
   }

   public int getNumberOfWaypoints()
   {
      return waypoints.length;
   }

   public void getControlFramePose(RigidBody endEffector, FramePose controlFramePoseToPack)
   {
      ReferenceFrame referenceFrame = endEffector == null ? null : endEffector.getBodyFixedFrame();
      controlFramePoseToPack.setToZero(referenceFrame);

      if (controlFramePositionInEndEffector != null)
         controlFramePoseToPack.setPosition(controlFramePositionInEndEffector);
      if (controlFrameOrientationInEndEffector != null)
         controlFramePoseToPack.setOrientation(controlFrameOrientationInEndEffector);
   }

   @Override
   public boolean epsilonEquals(WaypointBasedTrajectoryMessage other, double epsilon)
   {
      if (getNumberOfWaypoints() != other.getNumberOfWaypoints())
         return false;

      if (!ArrayTools.deltaEquals(waypointTimes, other.waypointTimes, epsilon))
         return false;

      for (int i = 0; i < getNumberOfWaypoints(); i++)
      {
         if (!waypoints[i].epsilonEquals(other.waypoints[i], epsilon))
            return false;
      }

      if (!angularSelectionMatrix.epsilonEquals(other.angularSelectionMatrix, epsilon))
         return false;
      if (!linearSelectionMatrix.epsilonEquals(other.linearSelectionMatrix, epsilon))
         return false;
      
      if(weight != other.weight)
         return false;

      return true;
   }
}

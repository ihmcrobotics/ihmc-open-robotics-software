package us.ihmc.humanoidRobotics.communication.packets.walking;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packets.IHMCRosApiMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.VisualizablePacket;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

@ClassDocumentation("This message commands the controller first to unload if necessary and then to move in taskspace a foot to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired foot pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class FootTrajectoryMessage extends IHMCRosApiMessage<FootTrajectoryMessage> implements TransformableDataObject<FootTrajectoryMessage>, VisualizablePacket
{
   @FieldDocumentation("Specifies the which foot will execute the trajectory.")
   public RobotSide robotSide;
   @FieldDocumentation("List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public SE3TrajectoryPointMessage[] taskspaceTrajectoryPoints;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public FootTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param footTrajectoryMessage message to clone.
    */
   public FootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      setUniqueId(footTrajectoryMessage.getUniqueId());
      setDestination(footTrajectoryMessage.getDestination());
      robotSide = footTrajectoryMessage.robotSide;
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[footTrajectoryMessage.getNumberOfTrajectoryPoints()];
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(footTrajectoryMessage.taskspaceTrajectoryPoints[i]);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired foot position expressed in world frame.
    * @param desiredOrientation desired foot orientation expressed in world frame.
    */
   public FootTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      Vector3d zeroLinearVelocity = new Vector3d();
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[] {new SE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3d, Quat4d, Vector3d, Vector3d)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which foot is performing the trajectory.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public FootTrajectoryMessage(RobotSide robotSide, int numberOfTrajectoryPoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[numberOfTrajectoryPoints];
   }

   public void setTrajectoryPoint(int trajectoryPointIndex, double time, Point3d position, Quat4d orientation, Vector3d linearVelocity, Vector3d angularVelocity)
   {
      rangeCheck(trajectoryPointIndex);
      taskspaceTrajectoryPoints[trajectoryPointIndex] = new SE3TrajectoryPointMessage(time, position, orientation, linearVelocity, angularVelocity);
   }

   public int getNumberOfTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints.length;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public SE3TrajectoryPointMessage getTrajectoryPoint(int trajectoryPointIndex)
   {
      rangeCheck(trajectoryPointIndex);
      return taskspaceTrajectoryPoints[trajectoryPointIndex];
   }

   public SE3TrajectoryPointMessage[] getTrajectoryPoints()
   {
      return taskspaceTrajectoryPoints;
   }

   private void rangeCheck(int trajectoryPointIndex)
   {
      if (trajectoryPointIndex >= getNumberOfTrajectoryPoints() || trajectoryPointIndex < 0)
         throw new IndexOutOfBoundsException("Trajectory point index: " + trajectoryPointIndex + ", number of trajectory points: " + getNumberOfTrajectoryPoints());
   }

   @Override
   public boolean epsilonEquals(FootTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      if (getNumberOfTrajectoryPoints() != other.getNumberOfTrajectoryPoints())
         return false;

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
      {
         if (!taskspaceTrajectoryPoints[i].epsilonEquals(other.taskspaceTrajectoryPoints[i], epsilon))
            return false;
      }

      return true;
   }

   @Override
   public FootTrajectoryMessage transform(RigidBodyTransform transform)
   {
      FootTrajectoryMessage transformedFootTrajectoryMessage = new FootTrajectoryMessage(robotSide, getNumberOfTrajectoryPoints());

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         transformedFootTrajectoryMessage.taskspaceTrajectoryPoints[i] = taskspaceTrajectoryPoints[i].transform(transform);

      return transformedFootTrajectoryMessage;
   }

   @Override
   public String toString()
   {
      String ret = "";
      if (taskspaceTrajectoryPoints != null)
         ret = "Foot SE3 trajectory: number of SE3 trajectory points = " + getNumberOfTrajectoryPoints();
      else
         ret = "Foot SE3 trajectory: no SE3 trajectory points";

      return ret + ", robotSide = " + robotSide + ".";
   }
}

package us.ihmc.humanoidRobotics.communication.packets.manipulation;

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
import us.ihmc.tools.DocumentedEnum;

@ClassDocumentation("This message commands the controller to move in taskspace a hand to the desired pose (position & orientation) while going through the specified trajectory points."
      + " A third order polynomial function is used to interpolate positions and a hermite based curve (third order) is used to interpolate the orientations."
      + " To excute a single straight line trajectory to reach a desired hand pose, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.")
public class HandTrajectoryMessage extends IHMCRosApiMessage<HandTrajectoryMessage> implements TransformableDataObject<HandTrajectoryMessage>, VisualizablePacket
{
   public enum BaseForControl implements DocumentedEnum<BaseForControl>
   {
      CHEST, WORLD;

      @Override
      public String getDocumentation(BaseForControl var)
      {
         switch (var)
         {
         case CHEST:
            return "The hand is controlled with respect to the chest. In other words, the controlled hand moves along with the chest.";
         case WORLD:
            return "The hand is controlled with respect to the estimated world. In other words, the controlled hand will remain fixed in world even if the robot starts moving.";

         default:
            return "No documentation available.";
         }
      }

      @Override
      public BaseForControl[] getDocumentedValues()
      {
         return values();
      }
   }

   @FieldDocumentation("Specifies which hand will execute the trajectory.")
   public RobotSide robotSide;
   @FieldDocumentation("Specifies whether the pose should be held with respect to the world or the chest. Note that in any case the desired hand pose must be expressed in world frame.")
   public BaseForControl base;
   @FieldDocumentation("List of trajectory points (in taskpsace) to go through while executing the trajectory. All the information contained in these trajectory points needs to be expressed in world frame.")
   public SE3TrajectoryPointMessage[] taskspaceTrajectoryPoints;

   /**
    * Empty constructor for serialization.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandTrajectoryMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   /**
    * Clone constructor.
    * @param handTrajectoryMessage message to clone.
    */
   public HandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      setUniqueId(handTrajectoryMessage.getUniqueId());
      setDestination(handTrajectoryMessage.getDestination());
      robotSide = handTrajectoryMessage.robotSide;
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[handTrajectoryMessage.getNumberOfTrajectoryPoints()];
      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         taskspaceTrajectoryPoints[i] = new SE3TrajectoryPointMessage(handTrajectoryMessage.taskspaceTrajectoryPoints[i]);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace. The chest is used as the base for the control.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      this(robotSide, BaseForControl.CHEST, trajectoryTime, desiredPosition, desiredOrientation);
   }

   /**
    * Use this constructor to execute a straight line trajectory in taskspace.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param base define with respect to what base the hand is controlled.
    * @param trajectoryTime how long it takes to reach the desired pose.
    * @param desiredPosition desired hand position expressed in world frame.
    * @param desiredOrientation desired hand orientation expressed in world frame.
    */
   public HandTrajectoryMessage(RobotSide robotSide, BaseForControl base, double trajectoryTime, Point3d desiredPosition, Quat4d desiredOrientation)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.base = base;
      Vector3d zeroLinearVelocity = new Vector3d();
      Vector3d zeroAngularVelocity = new Vector3d();
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[] {new SE3TrajectoryPointMessage(trajectoryTime, desiredPosition, desiredOrientation, zeroLinearVelocity, zeroAngularVelocity)};
   }

   /**
    * Use this constructor to build a message with more than one trajectory point.
    * This constructor only allocates memory for the trajectory points, you need to call {@link #setTrajectoryPoint(int, double, Point3d, Quat4d, Vector3d, Vector3d)} for each trajectory point afterwards.
    * Set the id of the message to {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    * @param robotSide is used to define which hand is performing the trajectory.
    * @param base define with respect to what base the hand is controlled.
    * @param numberOfTrajectoryPoints number of trajectory points that will be sent to the controller.
    */
   public HandTrajectoryMessage(RobotSide robotSide, BaseForControl base, int numberOfTrajectoryPoints)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.robotSide = robotSide;
      this.base = base;
      taskspaceTrajectoryPoints = new SE3TrajectoryPointMessage[numberOfTrajectoryPoints];
   }

   /**
    * Create a trajectory point.
    * @param trajectoryPointIndex index of the trajectory point to create.
    * @param time time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
    * @param position define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.
    * @param orientation define the desired 3D orientation to be reached at this trajectory point. It is expressed in world frame.
    * @param linearVelocity define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.
    * @param angularVelocity define the desired 3D angular velocity to be reached at this trajectory point. It is expressed in world frame.
    */
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

   public BaseForControl getBase()
   {
      return base;
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
   public boolean epsilonEquals(HandTrajectoryMessage other, double epsilon)
   {
      if (robotSide != other.robotSide)
         return false;
      if (base != other.base)
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
   public HandTrajectoryMessage transform(RigidBodyTransform transform)
   {
      HandTrajectoryMessage transformedHandTrajectoryMessage = new HandTrajectoryMessage(robotSide, base, getNumberOfTrajectoryPoints());

      for (int i = 0; i < getNumberOfTrajectoryPoints(); i++)
         transformedHandTrajectoryMessage.taskspaceTrajectoryPoints[i] = taskspaceTrajectoryPoints[i].transform(transform);

      return transformedHandTrajectoryMessage;
   }

   @Override
   public String toString()
   {
      String ret = "";
      if (taskspaceTrajectoryPoints != null)
         ret = "Hand SE3 trajectory: number of SE3 trajectory points = " + getNumberOfTrajectoryPoints();
      else
         ret = "Hand SE3 trajectory: no SE3 trajectory points";

      return ret + ", robotSide = " + robotSide + ", base for control = " + base;
   }
}

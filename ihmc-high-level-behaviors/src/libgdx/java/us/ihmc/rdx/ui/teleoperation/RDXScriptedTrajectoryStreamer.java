package us.ihmc.rdx.ui.teleoperation;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXScriptedTrajectoryStreamer
{
   private final double trajectoryTime;
   private boolean isDone = false;
   private final SideDependentList<FramePoint3D> handPositions = new SideDependentList<>(side -> new FramePoint3D());

   public RDXScriptedTrajectoryStreamer(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   public boolean isDone()
   {
      return isDone;
   }

   public void handCirclesMotion(double t)
   {
      if (t > trajectoryTime)
      {
         isDone = true;
         return;
      }
      else
      {
         isDone = false;
      }

      double circleRadius = 0.25;
      double circleFrequency = 0.25;
      SideDependentList<Point3D> circleCenters = new SideDependentList<>(side -> new Point3D(0.3, side.negateIfRightSide(0.225), 0.0));
      SideDependentList<Vector3D> circleCenterVelocities = new SideDependentList<>(side -> side == RobotSide.LEFT ?
            new Vector3D(0.0, 0.0, 0.0) :
            new Vector3D());

      //      for (double t = 0.0; t < trajectoryTime; t += streamPeriod)

      for (RobotSide robotSide : RobotSide.values)
      {
         handPositions.put(robotSide,
                           circlePositionAt(t,
                                            robotSide.negateIfRightSide(circleFrequency),
                                            circleRadius,
                                            circleCenters.get(robotSide),
                                            circleCenterVelocities.get(robotSide)));
      }
   }

   public FramePoint3D getHandPosition(RobotSide robotSide, double time)
   {
      handCirclesMotion(time);
      return handPositions.get(robotSide);
   }

   public static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center)
   {
      return circlePositionAt(time, frequency, radius, center, new Vector3D());
   }

   public static FramePoint3D circlePositionAt(double time, double frequency, double radius, Point3DReadOnly center, Vector3DReadOnly centerVelocity)
   {
      double angle = 2.0 * Math.PI * frequency * time;
      Vector3D offset = new Vector3D(Axis3D.Z);
      offset.scale(radius);
      RotationMatrixTools.applyRollRotation(angle, offset, offset);
      FramePoint3D position = new FramePoint3D();
      position.add(center, offset);
      position.scaleAdd(time, centerVelocity, position);
      return position;
   }
}

//         KinematicsStreamingToolboxInputMessage input = new KinematicsStreamingToolboxInputMessage();
//         //         input.getInputs().add().set(KinematicsToolboxMessageFactory.holdRigidBodyCurrentPose(fullRobotModelAtInitialConfiguration.getPelvis()));
//
//         for (RobotSide robotSide : RobotSide.values)
//         {
//            handPositions.put(robotSide, circlePositionAt(t,
//                                                          robotSide.negateIfRightSide(circleFrequency),
//                                                          circleRadius,
//                                                          circleCenters.get(robotSide),
//                                                          circleCenterVelocities.get(robotSide)));
//            FramePoint3D position = circlePositionAt(t,
//                                                     robotSide.negateIfRightSide(circleFrequency),
//                                                     circleRadius,
//                                                     circleCenters.get(robotSide),
//                                                     circleCenterVelocities.get(robotSide));
//            KinematicsToolboxRigidBodyMessage message = MessageTools.createKinematicsToolboxRigidBodyMessage(syncedRobot.getFullRobotModel().getHand(robotSide),
//                                                                                                             position);
//            input.getInputs().add().set(message);
//         }
//
//         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName()), input);
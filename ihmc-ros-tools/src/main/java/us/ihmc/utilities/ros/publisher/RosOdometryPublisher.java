package us.ihmc.utilities.ros.publisher;

import org.ros.message.Time;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;
import geometry_msgs.TwistWithCovariance;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import std_msgs.Header;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.utilities.ros.RosTools;

public class RosOdometryPublisher extends RosTopicPublisher<nav_msgs.Odometry>
{
   private int counter=0;
   public RosOdometryPublisher(boolean latched)
   {
      super(nav_msgs.Odometry._TYPE,latched);
   }
   
   public void publish(long timestamp, RigidBodyTransform transform, Vector3D32 linearVelocity, Vector3D32 angularVelocity, String childFrame, String frameId)
   {
      Odometry message = getMessage();

      Header header = createHeaderMsg(timestamp);
      header.setFrameId(frameId);
      message.setHeader(header);
      
      PoseWithCovariance poseWithCovariance = createPoseWithCovarianceMsg(transform);
      message.setPose(poseWithCovariance);
      
      TwistWithCovariance twistWithCovariance = createTwistWithCovariance(linearVelocity, angularVelocity);
      message.setTwist(twistWithCovariance);
      
      message.setChildFrameId(childFrame);
      
      publish(message);
   }

   private TwistWithCovariance createTwistWithCovariance(Vector3D32 linearVelocity, Vector3D32 angularVelocity)
   {
      TwistWithCovariance twistWithCovariance = newMessageFromType(TwistWithCovariance._TYPE);
      Twist twist = createTwistMsg(linearVelocity, angularVelocity);
      twistWithCovariance.setTwist(twist);
      return twistWithCovariance;
   }

   private Twist createTwistMsg(Vector3D32 linearVelocity, Vector3D32 angularVelocity)
   {
      Twist twist = newMessageFromType(Twist._TYPE);
      
      Vector3 angularVelocityAsRosVector = getVector3(angularVelocity);
      twist.setAngular(angularVelocityAsRosVector);
      
      Vector3 linearVelocityAsRosVector = getVector3(linearVelocity);
      twist.setLinear(linearVelocityAsRosVector);
      
      return twist;
   }

   private Vector3 getVector3(Vector3D32 angularVelocity)
   {
      Vector3 rosVector3 = newMessageFromType(Vector3._TYPE);
      RosTools.packVector3fToGeometry_msgsVector3(angularVelocity, rosVector3);
      return rosVector3;
   }

   private Header createHeaderMsg(long timestamp)
   {
      Header header = newMessageFromType(Header._TYPE);
      header.setStamp(Time.fromNano(timestamp));
      header.setSeq(counter);
      counter++;
      return header;
   }

   private PoseWithCovariance createPoseWithCovarianceMsg(RigidBodyTransform transform)
   {
      PoseWithCovariance poseWithCovariance = newMessageFromType(PoseWithCovariance._TYPE);
      Pose pose = createPoseMsg(transform);
      poseWithCovariance.setPose(pose);
      return poseWithCovariance;
   }

   private Pose createPoseMsg(RigidBodyTransform transform)
   {
      Pose pose = newMessageFromType(geometry_msgs.Pose._TYPE);
      Point point = newMessageFromType(Point._TYPE);
      Quaternion rotation = newMessageFromType(Quaternion._TYPE);
      pose.setPosition(point);
      pose.setOrientation(rotation);
      RosTools.packRigidBodyTransformToGeometry_msgsPose(transform, pose);
      return pose;
   }
}
package us.ihmc.utilities.ros.publisher;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.internal.message.topic.TopicMessageFactory;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.Time;

import std_msgs.Header;
import tf.tfMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class RosTf1Publisher extends RosTopicPublisher<tf.tfMessage> implements RosTfPublisherInterface
{
   private TopicMessageFactory topicMessageFactory;
   private int seq;
   public RosTf1Publisher(boolean latched)
   {
      super(tf.tfMessage._TYPE, latched);
      MessageDefinitionProvider messageDefinitionProvider = new MessageDefinitionReflectionProvider();
      topicMessageFactory = new TopicMessageFactory(messageDefinitionProvider);
   }
   
   @Override
   public void connected()
   {
   }
   
   private Transform getRosTransform(RigidBodyTransform transform3d)
   {
      Transform transform = topicMessageFactory.newFromType(Transform._TYPE);
      Quaternion rot = topicMessageFactory.newFromType(Quaternion._TYPE);
      Vector3 trans = topicMessageFactory.newFromType(Vector3._TYPE);
      
      Quat4d quat4d = new Quat4d();
      Vector3d vector3d = new Vector3d();
      transform3d.get(quat4d, vector3d);
      
      rot.setW(quat4d.getW());
      rot.setX(quat4d.getX());
      rot.setY(quat4d.getY());
      rot.setZ(quat4d.getZ());
      
      transform.setRotation(rot);
      
      trans.setX(vector3d.getX());
      trans.setY(vector3d.getY());
      trans.setZ(vector3d.getZ());
      
      transform.setTranslation(trans);
      
      return transform;
   }
   
   public void publish(RigidBodyTransform transform3d, long timeStamp, String parentFrame, String childFrame)
   {
      TransformStamped transformStamped = topicMessageFactory.newFromType(TransformStamped._TYPE);
      Transform transform = getRosTransform(transform3d);
      transformStamped.setTransform(transform);
      transformStamped.setChildFrameId(childFrame);
      Header header = transformStamped.getHeader();
      header.setStamp(Time.fromNano(timeStamp));
      header.setFrameId(parentFrame);
      header.setSeq(seq++);
      transformStamped.setHeader(header);
      
      
      tfMessage message = getMessage();
      
      List<TransformStamped> tfs = new ArrayList<TransformStamped>();
      tfs.add(transformStamped);
      message.setTransforms(tfs);
      publish(message);
   }
}

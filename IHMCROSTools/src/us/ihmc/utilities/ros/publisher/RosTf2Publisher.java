package us.ihmc.utilities.ros.publisher;

import java.util.ArrayList;
import java.util.List;

import org.ros.internal.message.definition.MessageDefinitionReflectionProvider;
import org.ros.internal.message.topic.TopicMessageFactory;
import org.ros.message.MessageDefinitionProvider;
import org.ros.message.Time;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import std_msgs.Header;
import tf2_msgs.TFMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class RosTf2Publisher extends RosTopicPublisher<tf2_msgs.TFMessage> implements RosTfPublisherInterface
{
   private TopicMessageFactory topicMessageFactory;
   private int seq;
   public RosTf2Publisher(boolean latched)
   {
      super(tf2_msgs.TFMessage._TYPE, latched);
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
      
      us.ihmc.euclid.tuple4D.Quaternion quat4d = new us.ihmc.euclid.tuple4D.Quaternion();
      Vector3D vector3d = new Vector3D();
      transform3d.get(quat4d, vector3d);
      
      rot.setW(quat4d.getS());
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
      
      
      TFMessage message = getMessage();
      
      List<TransformStamped> tfs = new ArrayList<TransformStamped>();
      tfs.add(transformStamped);
      message.setTransforms(tfs);
      publish(message);
   }
}

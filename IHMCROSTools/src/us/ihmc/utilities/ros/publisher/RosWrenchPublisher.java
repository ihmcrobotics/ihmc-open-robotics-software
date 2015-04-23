package us.ihmc.utilities.ros.publisher;

import geometry_msgs.Vector3;

import org.ejml.data.DenseMatrix64F;


public class RosWrenchPublisher extends RosTopicPublisher<geometry_msgs.Wrench>
{
   public RosWrenchPublisher(boolean latched)
   {
      super(geometry_msgs.Wrench._TYPE,latched);
   }

   public void publish(long timeStamp, float[] footSensorWrench)
   {
      /* FROM AtlasSensorReader
      temporaryWrench.set(0, wrist_sensor.getM().getX() - offset.get(0));
      temporaryWrench.set(1, wrist_sensor.getM().getY() - offset.get(1));
      temporaryWrench.set(2, wrist_sensor.getM().getZ() - offset.get(2));
      temporaryWrench.set(3, wrist_sensor.getF().getX() - offset.get(3));
      temporaryWrench.set(4, wrist_sensor.getF().getY() - offset.get(4));
      temporaryWrench.set(5, wrist_sensor.getF().getZ() - offset.get(5));
       */
      
      geometry_msgs.Wrench message = getMessage();
      Vector3  force = newMessageFromType(Vector3._TYPE);
      Vector3  torque = newMessageFromType(Vector3._TYPE);
      
      torque.setX(footSensorWrench[0]);
      torque.setY(footSensorWrench[1]);
      torque.setZ(footSensorWrench[2]);
      
      force.setX(footSensorWrench[3]);
      force.setY(footSensorWrench[4]);
      force.setZ(footSensorWrench[5]);
      
      message.setTorque(torque);
      message.setForce(force);
      publish(message);
   }
}
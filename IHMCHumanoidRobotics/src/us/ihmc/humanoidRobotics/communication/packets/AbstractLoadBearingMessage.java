package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;

public class AbstractLoadBearingMessage <T extends AbstractLoadBearingMessage<T>> extends TrackablePacket<T>
{
   /** If set to true this will load the contact point. Otherwise the rigid body will stop bearing load. */
   public boolean load = false;

   /** Sets the coefficient of friction that the controller will use for the contact point. */
   public double coefficientOfFriction = 0.0;

   /** Sets the position of the contact point in the frame of the end effector body. */
   public Point3D contactPointInBodyFrame = new Point3D();

   /** Sets the contact normal used by the controller to load the contact point. */
   public Vector3D contactNormalInWorldFrame = new Vector3D();

   public AbstractLoadBearingMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public AbstractLoadBearingMessage(Random random)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      load = random.nextBoolean();
      coefficientOfFriction = random.nextDouble();
      contactPointInBodyFrame = EuclidCoreRandomTools.generateRandomPoint3D(random);
      contactNormalInWorldFrame = EuclidCoreRandomTools.generateRandomVector3D(random);
   }

   public void setLoad(boolean load)
   {
      this.load = load;
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction = coefficientOfFriction;
   }

   public void setContactPointInBodyFrame(Point3D contactPointInBodyFrame)
   {
      this.contactPointInBodyFrame.set(contactPointInBodyFrame);
   }

   public void setContactNormalInWorldFrame(Vector3D contactNormalInWorldFrame)
   {
      this.contactNormalInWorldFrame.set(contactNormalInWorldFrame);
   }

   public boolean getLoad()
   {
      return load;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public Point3D getContactPointInBodyFrame()
   {
      return contactPointInBodyFrame;
   }

   public Vector3D getContactNormalInWorldFrame()
   {
      return contactNormalInWorldFrame;
   }

   @Override
   public boolean epsilonEquals(T other, double epsilon)
   {
      if (load != other.load)
         return false;
      if (!(MathTools.epsilonEquals(coefficientOfFriction, other.coefficientOfFriction, epsilon)))
         return false;
      if (!contactPointInBodyFrame.epsilonEquals(other.contactPointInBodyFrame, epsilon))
         return false;
      if (!contactNormalInWorldFrame.epsilonEquals(other.contactNormalInWorldFrame, epsilon))
         return false;
      return true;
   }

}

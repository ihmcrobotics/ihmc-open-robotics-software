package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.TrackablePacket;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;

public class AbstractLoadBearingMessage <T extends AbstractLoadBearingMessage<T>> extends TrackablePacket<T>
{
   /** If set to true this will load the contact point. Otherwise the rigid body will stop bearing load. */
   public boolean load = false;

   /** Sets the coefficient of friction that the controller will use for the contact point. */
   public double coefficientOfFriction = 0.0;

   /** Sets the transform of the contact frame in the frame of the end effector body. */
   public RigidBodyTransform bodyFrameToContactFrame = new RigidBodyTransform();

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
      bodyFrameToContactFrame = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
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

   public void setBodyFrameToContactFrame(RigidBodyTransform bodyFrameToContactFrame)
   {
      this.bodyFrameToContactFrame.set(bodyFrameToContactFrame);
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

   public RigidBodyTransform getBodyFrameToContactFrame()
   {
      return bodyFrameToContactFrame;
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
      if (!bodyFrameToContactFrame.epsilonEquals(other.bodyFrameToContactFrame, epsilon))
         return false;
      if (!contactNormalInWorldFrame.epsilonEquals(other.contactNormalInWorldFrame, epsilon))
         return false;
      return true;
   }

}

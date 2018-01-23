package us.ihmc.robotics.math.frames;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;

public class YoTwist extends YoSpatialVector
{
   /** This is only for assistance. The data is stored in the YoVariables, not in here! */
   private final Twist twist;
   /** Redundant but allows to make sure the frame isn't changed. */
   protected final ReferenceFrame bodyFrame;
   protected final ReferenceFrame baseFrame;
   
   public YoTwist(String namePrefix, String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, expressedInFrame, registry);
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.twist = createEmptyTwist();
   }
   
   public YoTwist(String namePrefix, ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", bodyFrame, baseFrame, expressedInFrame, registry);
   }
   
   public YoTwist(YoFrameVector yoLinearVelocity, YoFrameVector yoAngularVelocity, ReferenceFrame bodyFrame, ReferenceFrame baseFrame)
   {
      super(yoLinearVelocity, yoAngularVelocity);
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.twist = createEmptyTwist();
   }

   private Twist createEmptyTwist()
   {
      return new Twist(bodyFrame, baseFrame, expressedInFrame);
   }
   
   protected void putYoValuesIntoTwist()
   {
      twist.setToZero(bodyFrame, baseFrame, expressedInFrame);
      twist.setLinearPart(linearPart);
      twist.setAngularPart(angularPart);
   }
   
   protected void getYoValuesFromTwist()
   {
      linearPart.set(twist.getExpressedInFrame(), twist.getLinearPartX(), twist.getLinearPartY(), twist.getLinearPartZ());
      angularPart.set(twist.getExpressedInFrame(), twist.getAngularPartX(), twist.getAngularPartY(), twist.getAngularPartZ());
   }
   
   public void set(Twist twist)
   {
      this.twist.checkAndSet(twist);
      getYoValuesFromTwist();
   }
   
   public void normalize()
   {
      putYoValuesIntoTwist();
      twist.normalize();
      getYoValuesFromTwist();
   }
   
   public void add(Twist twist)
   {
      putYoValuesIntoTwist();
      this.twist.add(twist);
      getYoValuesFromTwist();
   }
   
   public void sub(Twist twist)
   {
      putYoValuesIntoTwist();
      this.twist.sub(twist);
      getYoValuesFromTwist();
   }
   
   public double dot(Wrench wrench)
   {
      putYoValuesIntoTwist();
      return twist.dot(wrench);
   }

   public Twist getTwist()
   {
      putYoValuesIntoTwist();
      return twist;
   }

   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   public ReferenceFrame getBaseFrame()
   {
      return baseFrame;
   }
}

package us.ihmc.robotics.math.frames;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;

public class YoWrench extends YoSpatialVector
{
   /** This is only for assistance. The data is stored in the YoVariables, not in here! */
   private final Wrench wrench;
   /** Redundant but allows to make sure the frame isn't changed. */
   protected final ReferenceFrame bodyFrame;
   
   public YoWrench(YoFrameVector yoForce, YoFrameVector yoTorque, ReferenceFrame bodyFrame)
   {
      super(yoForce, yoTorque);
      this.bodyFrame = bodyFrame;
      this.wrench = createEmptyWrench();
   }

   public YoWrench(String namePrefix, String nameSuffix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, expressedInFrame, registry);
      this.bodyFrame = bodyFrame;
      this.wrench = createEmptyWrench();
   }

   public YoWrench(String namePrefix, ReferenceFrame bodyFrame, ReferenceFrame expressedInFrame, YoVariableRegistry registry)
   {
      this(namePrefix, "", bodyFrame, expressedInFrame, registry);
   }

   private Wrench createEmptyWrench()
   {
      return new Wrench(bodyFrame, expressedInFrame);
   }
   
   protected void putYoValuesIntoWrench()
   {
      wrench.setToZero(bodyFrame, expressedInFrame);
      wrench.setLinearPart(linearPart);
      wrench.setAngularPart(angularPart);
   }
   
   protected void getYoValuesFromWrench()
   {
      linearPart.set(wrench.getExpressedInFrame(), wrench.getLinearPartX(), wrench.getLinearPartY(), wrench.getLinearPartZ());
      angularPart.set(wrench.getExpressedInFrame(), wrench.getAngularPartX(), wrench.getAngularPartY(), wrench.getAngularPartZ());
   }
   
   public void set(Wrench wrench)
   {
      this.wrench.checkAndSet(wrench);
      getYoValuesFromWrench();
   }
   
   public void add(Wrench wrench)
   {
      putYoValuesIntoWrench();
      this.wrench.add(wrench);
      getYoValuesFromWrench();
   }
   
   public void sub(Wrench wrench)
   {
      putYoValuesIntoWrench();
      this.wrench.sub(wrench);
      getYoValuesFromWrench();
   }
   
   public double dot(Twist twist)
   {
      putYoValuesIntoWrench();
      return wrench.dot(twist);
   }

   public Wrench getWrench()
   {
      putYoValuesIntoWrench();
      return wrench;
   }

   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }
}

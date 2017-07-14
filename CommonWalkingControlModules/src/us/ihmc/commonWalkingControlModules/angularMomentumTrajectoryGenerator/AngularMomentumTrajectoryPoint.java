package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AngularMomentumTrajectoryPoint
{
   protected YoFrameVector angularMomentum;
   private YoFrameVector torque;
   private YoFrameVector rotatum;
   private YoDouble time;
   
   public AngularMomentumTrajectoryPoint(String namePrefix, YoVariableRegistry registry, ReferenceFrame referenceFrame)   
   {
      time = new YoDouble(namePrefix + "Time", registry);
      angularMomentum = new YoFrameVector(namePrefix + "AngMom", referenceFrame, registry);
      torque = new YoFrameVector(namePrefix + "Torque", referenceFrame, registry);
      rotatum = new YoFrameVector(namePrefix + "Rotatum", referenceFrame, registry);
   }
   
   public void set(AngularMomentumTrajectoryPoint angMomTrajPoint)
   {
      this.angularMomentum.set(angMomTrajPoint.angularMomentum);
      this.torque.set(angMomTrajPoint.torque);
      this.rotatum.set(angMomTrajPoint.rotatum);
      this.time.set(angMomTrajPoint.time.getDoubleValue());
   }
   
   public void setAngularMomentum(YoFrameVector vector)
   {
      this.angularMomentum.set(vector);
   }
   
   public void setAngularMomentum(FrameVector vector)
   {
      this.angularMomentum.set(vector);
   }
   
   public void setTorque(YoFrameVector vector)
   {
      this.torque.set(vector);
   }
   
   public void setTorque(FrameVector vector)
   {
      this.torque.set(vector);
   }
   
   public void setRotatum(YoFrameVector vector)
   {
      this.rotatum.set(vector);
   }

   public void setRotatum(FrameVector vector)
   {
      this.rotatum.set(vector);
   }

   public void setTime(double time)
   {
      this.time.set(time);
   }
   
   public void setAngularMomentum(double x, double y, double z)
   {
      this.angularMomentum.set(x, y, z);
   }
   
   public void reset()
   {
      this.angularMomentum.setToZero();
      this.time.set(0);
   }
   
   public FrameVector getAngularMomentum()
   {
      return angularMomentum.getFrameTuple();
   }
   
   public FrameVector getTorque()
   {
      return torque.getFrameTuple();
   }

   public FrameVector getRotatum()
   {
      return rotatum.getFrameTuple();
   }

   public void getAngularMomentum(FrameVector angMomToPack)
   {
      angMomToPack.setIncludingFrame(angularMomentum.getFrameTuple());
   }

   public void getTorque(FrameVector torqueToPack)
   {
      torqueToPack.setIncludingFrame(torque.getFrameTuple());
   }

   public void getRotatum(FrameVector rotatumToPack)
   {
      rotatumToPack.setIncludingFrame(rotatum.getFrameTuple());
   }
}


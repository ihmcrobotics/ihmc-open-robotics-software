package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class ContactState
{
   private ContactType state;
   private double duration;
   private final FramePose3D pose;
   private final FrameVector3D linearVelocity;
   private final FrameVector3D angularVelocity;
   private final FrameVector3D groundReactionForce;
   private final FrameVector3D torque;
   private final ConvexPolygon2D supportPolygon;

   public ContactState()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public ContactState(ReferenceFrame referenceFrame)
   {
      this.pose = new FramePose3D(referenceFrame);
      this.linearVelocity = new FrameVector3D(referenceFrame);
      this.angularVelocity = new FrameVector3D(referenceFrame);
      this.groundReactionForce = new FrameVector3D(referenceFrame);
      this.torque = new FrameVector3D(referenceFrame);
      this.supportPolygon = new ConvexPolygon2D();
      reset();
   }

   public void reset()
   {
      state = null;
      duration = Double.NaN;
      supportPolygon.clear();
   }

   public void setContactType(ContactType stateToSet)
   {
      this.state = stateToSet;
   }

   public void setSupportPolygon(ConvexPolygon2D supportPolygonToSet)
   {
      this.supportPolygon.set(supportPolygonToSet);
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   public void getSupportPolygon(ConvexPolygon2D supportPolygonToSet)
   {
      supportPolygonToSet.set(supportPolygon);
   }

   public ContactType getContactType()
   {
      return state;
   }

   public double getDuration()
   {
      return duration;
   }

   public void getCoMPosition(FramePoint3D position)
   {
      position.setIncludingFrame(pose.getPosition());
   }
   
   public void setCoMPosition(FramePoint3D positionToSet)
   {
      this.pose.setPosition(positionToSet);
   }

   public void getCoMLinearVelocity(FrameVector3D linearVelocity)
   {
      linearVelocity.setIncludingFrame(this.linearVelocity);
   }
   
   public void setCoMLinearVelocity(FrameVector3D angularVelocityToSet)
   {
      this.angularVelocity.setIncludingFrame(angularVelocityToSet);
   }

   public void getCoMGroundReaction(FrameVector3D groundReactionForce)
   {
      groundReactionForce.setIncludingFrame(this.groundReactionForce);
   }
   
   public void setCoMGroundReaction(FrameVector3D groundReactionToSet)
   {
      this.groundReactionForce.setIncludingFrame(groundReactionToSet);
   }

   public void getCoMOrientation(FrameQuaternion comOrientation)
   {
      comOrientation.setIncludingFrame(pose.getOrientation());
   }
   
   public void setCoMOrientation(FrameQuaternion comOrientationToSet)
   {
      this.pose.setOrientation(comOrientationToSet);
   }

   public void getCoMAngularVelocity(FrameVector3D angularVelocity)
   {
      angularVelocity.setIncludingFrame(this.angularVelocity);
   }
   
   public void setCoMAngularVelocity(FrameVector3D angularVelocityToSet)
   {
      this.angularVelocity.setIncludingFrame(angularVelocityToSet);
   }

   public void getCoMTorque(FrameVector3D torque)
   {
      torque.setIncludingFrame(this.torque);
   }
   
   public void setCoMTorque(FrameVector3D torqueToSet)
   {
      this.torque.setIncludingFrame(torqueToSet);
   }
}
package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class EndEffector
{
   //TODO: YoVariableize the boolean?
   private OptimizerContactModel contactModel;
   private ReferenceFrame referenceFrame;
   private boolean loadBearing;

   public ReferenceFrame getFrame()
   {
      return this.referenceFrame;
   }

   public OptimizerContactModel getContactModel()
   {
      return contactModel;
   }

   public void setContactModel(OptimizerContactModel contactModel)
   {
      this.contactModel = contactModel;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public boolean isLoadBearing()
   {
      return loadBearing;
   }

   public void setLoadBearing(boolean inContact)
   {
      this.loadBearing = inContact;
   }

}

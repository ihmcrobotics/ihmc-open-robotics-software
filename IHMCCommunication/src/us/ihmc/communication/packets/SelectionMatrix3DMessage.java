package us.ihmc.communication.packets;

import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SelectionMatrix3DMessage extends Packet<SelectionMatrix3DMessage>
{
   @RosExportedField(documentation = "The Id of the reference frame defining the selection frame."
         + " When selecting the axes of interest, these axes refer to the selection frame axes."
         + " This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal to {@code 0L},"
         + " the selection matrix will then be generated regardless to what frame is it used in.")
   public long selectionFrameId = NameBasedHashCodeTools.NULL_HASHCODE;
   @RosExportedField(documentation = "Specifies whether the x-axis of the selection frame is an axis of interest.")
   public boolean xSelected = true;
   @RosExportedField(documentation = "Specifies whether the y-axis of the selection frame is an axis of interest.")
   public boolean ySelected = true;
   @RosExportedField(documentation = "Specifies whether the z-axis of the selection frame is an axis of interest.")
   public boolean zSelected = true;

   /**
    * Creates a new selection matrix message. It is initialized with all the axes selected. Until
    * the selection is changed, this selection matrix is independent from its selection frame.
    */
   public SelectionMatrix3DMessage()
   {
   }

   /**
    * Updates the selection of the axes of interest.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param xSelected whether the x-axis is an axis of interest.
    * @param ySelected whether the y-axis is an axis of interest.
    * @param zSelected whether the z-axis is an axis of interest.
    */
   public void setAxisSelection(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      this.xSelected = xSelected;
      this.ySelected = ySelected;
      this.zSelected = zSelected;
   }

   /**
    * Sets the selection frame such that the selection of the axes of interest now refers to the
    * axes of the given frame.
    * 
    * @param selectionFrame the new frame to which the axes selection is referring to.
    */
   public void setSelectionFrame(ReferenceFrame selectionFrame)
   {
      this.selectionFrameId = selectionFrame.getNameBasedHashCode();
   }

   /**
    * Sets the ID of the selection frame to use. The selection of the axes of interest refers to the
    * axes of the selection frame.
    * 
    * @param selectionFrame the ID of the new frame to which the axes selection is referring to.
    */
   public void setSelectionFrameId(long selectionFrameId)
   {
      this.selectionFrameId = selectionFrameId;
   }

   @Override
   public boolean epsilonEquals(SelectionMatrix3DMessage other, double epsilon)
   {
      if (selectionFrameId != other.selectionFrameId)
         return false;
      if (xSelected != other.xSelected)
         return false;
      if (ySelected != other.ySelected)
         return false;
      if (zSelected != other.zSelected)
         return false;

      return true;
   }
}

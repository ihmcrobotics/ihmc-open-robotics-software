package us.ihmc.communication.packets;

import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

@RosMessagePacket(documentation = "", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/selection_matrix")
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
    * Copy constructor.
    * 
    * @param selectionMatrix3D the original selection matrix to copy. Not modified.
    */
   public SelectionMatrix3DMessage(SelectionMatrix3DMessage selectionMatrix3D)
   {
      set(selectionMatrix3D);
   }

   /**
    * Sets this selection matrix message to {@code selectionMatrix3D}.
    * 
    * @param selectionMatrix3D the selection matrix to copy the data of. Not modified.
    */
   public void set(SelectionMatrix3D selectionMatrix3D)
   {
      setSelectionFrame(selectionMatrix3D.getSelectionFrame());
      xSelected = selectionMatrix3D.isXSelected();
      ySelected = selectionMatrix3D.isYSelected();
      zSelected = selectionMatrix3D.isZSelected();
   }

   @Override
   public void set(SelectionMatrix3DMessage other)
   {
      selectionFrameId = other.selectionFrameId;
      xSelected = other.xSelected;
      ySelected = other.ySelected;
      zSelected = other.zSelected;
      setPacketInformation(other);
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
      if (selectionFrame == null)
         selectionFrameId = NameBasedHashCodeTools.NULL_HASHCODE;
      else
         selectionFrameId = selectionFrame.getNameBasedHashCode();
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

   /**
    * Unpacks this message into the given {@code selectionMatrix3D}.
    * <p>
    * Note that the selection frame can not be retrieved here, it has to be set afterwards.
    * </p>
    * 
    * @param selectionMatrix3D the selection matrix into which this message is being unpacked.
    *           Modified.
    */
   public void getSelectionMatrix(SelectionMatrix3D selectionMatrix3D)
   {
      selectionMatrix3D.clearSelection();
      selectionMatrix3D.selectXAxis(xSelected);
      selectionMatrix3D.selectYAxis(ySelected);
      selectionMatrix3D.selectZAxis(zSelected);
   }

   public void setXSelected(boolean xSelected)
   {
      this.xSelected = xSelected;
   }

   public void setYSelected(boolean ySelected)
   {
      this.ySelected = ySelected;
   }

   public void setZSelected(boolean zSelected)
   {
      this.zSelected = zSelected;
   }

   public boolean getXSelected()
   {
      return xSelected;
   }

   public boolean getYSelected()
   {
      return ySelected;
   }

   public boolean getZSelected()
   {
      return zSelected;
   }

   /**
    * Returns the unique ID referring to the selection frame to use with this selection matrix.
    * 
    * @return the selection frame ID.
    */
   public long getSelectionFrameId()
   {
      return selectionFrameId;
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

package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * The {@code SelectionMatrix6D} provides a simple way to define for a given application what are
 * the axes of interest for spatial data consisting of an angular part and a linear part.
 * <p>
 * Given the set of axes of interest and a reference frame to which these axes refer to, the
 * {@code SelectionMatrix6D} is then able compute the corresponding 6-by-6 selection matrix:<br>
 * 
 * <pre>
 * S = / A<sub>3x3</sub> <b>0</b><sub>3x3</sub> \
 *     \ <b>0</b><sub>3x3</sub> L<sub>3x3</sub> /
 * </pre>
 * 
 * where <b>0</b><sub>3x3</sub> is the 3-by3 zero matrix, A<sub>3x3</sub> is the 3-by-3 selection
 * matrix for the angular part, and L<sub>3x3</sub> is the 3-by-3 selection matrix for the linear
 * part.
 * </p>
 * <p>
 * The principal use-case is for the controller core notably used for the walking controller. This
 * class can be used to clearly define what the axes to be controlled for a given end-effector.
 * </p>
 * <p>
 * Note that the {@link #selectionFrame} is optional. It is preferable to provide it when possible,
 * but when it is absent, i.e. equal to {@code null}, the selection matrix will then be generated
 * assuming the destination frame is the same as the selection frame.
 * </p>
 * <p>
 * Note that the two 3-by-3 selection matrices used by this class do not necessarily need to be
 * using the same selection frame.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class SelectionMatrix6D
{
   /**
    * The 3-by-3 selection matrix A<sub>3x3</sub> for the angular part of this selection matrix.
    */
   private final SelectionMatrix3D angularPart = new SelectionMatrix3D();
   /**
    * The 3-by-3 selection matrix L<sub>3x3</sub> for the linear part of this selection matrix.
    */
   private final SelectionMatrix3D linearPart = new SelectionMatrix3D();

   /**
    * Creates a new selection matrix. This selection matrix is initialized with all the axes
    * selected. Until the selection is changed, this selection matrix is independent from its
    * selection frame.
    */
   public SelectionMatrix6D()
   {
   }

   /**
    * Copy constructor.
    * 
    * @param other the selection matrix to copy. Not modified.
    */
   public SelectionMatrix6D(SelectionMatrix6D other)
   {
      set(other);
   }

   /**
    * Sets the selection frame of both the angular and linear parts to {@code null}.
    * <p>
    * When the selection frame is {@code null}, the conversion into a 6-by-6 selection matrix will
    * be done regardless of the destination frame.
    * </p>
    */
   public void clearSelectionFrame()
   {
      setSelectionFrame(null);
   }

   /**
    * Sets the selection frame of the angular part to {@code null}.
    * <p>
    * When the selection frame is {@code null}, the conversion into a selection matrix will be done
    * regardless of the destination frame.
    * </p>
    * <p>
    * The linear part of this selection matrix remains unchanged.
    * </p>
    */
   public void clearAngularSelectionFrame()
   {
      angularPart.clearSelectionFrame();
   }

   /**
    * Sets the selection frame of the linear part to {@code null}.
    * <p>
    * When the selection frame is {@code null}, the conversion into a selection matrix will be done
    * regardless of the destination frame.
    * </p>
    * <p>
    * The angular part of this selection matrix remains unchanged.
    * </p>
    */
   public void clearLinearSelectionFrame()
   {
      linearPart.clearSelectionFrame();
   }

   /**
    * Sets the selection frame such that the selection of the axes of interest now refers to the
    * axes of the given frame for both the angular and linear parts.
    * 
    * @param selectionFrame the new frame to which the axes selection is referring to.
    */
   public void setSelectionFrame(ReferenceFrame selectionFrame)
   {
      setSelectionFrames(selectionFrame, selectionFrame);
   }

   /**
    * Sets the selection frame for the angular and linear parts separately.
    * 
    * @param angularSelectionFrame the new frame to which the angular axes selection is referring
    *           to.
    * @param linearSelectionFrame the new frame to which the linear axes selection is referring to.
    */
   public void setSelectionFrames(ReferenceFrame angularSelectionFrame, ReferenceFrame linearSelectionFrame)
   {
      angularPart.setSelectionFrame(angularSelectionFrame);
      linearPart.setSelectionFrame(linearSelectionFrame);
   }

   /**
    * Selects all the axes and clears the selection frame.
    * <p>
    * Until the selection is changed, this selection matrix is independent from its selection frame.
    * </p>
    */
   public void resetSelection()
   {
      angularPart.resetSelection();
      linearPart.resetSelection();
   }

   /**
    * Selects all the angular axes and clears the angular selection frame.
    * <p>
    * The linear part remains unmodified.
    * </p>
    */
   public void resetAngularSelection()
   {
      angularPart.resetSelection();
   }

   /**
    * Selects all the linear axes and clears the linear selection frame.
    * <p>
    * The angular part remains unmodified.
    * </p>
    */
   public void resetLinearSelection()
   {
      linearPart.resetSelection();
   }

   /**
    * Deselects all the axes and clears the selection frames.
    * <p>
    * Until the selection is changed, this selection matrix is independent from its selection frame.
    * </p>
    */
   public void clearSelection()
   {
      angularPart.clearSelection();
      linearPart.clearSelection();
   }

   /**
    * Deselects all the angular axes and clears the angular selection frame.
    * <p>
    * The linear part remains unmodified.
    * </p>
    */
   public void clearAngularSelection()
   {
      angularPart.clearSelection();
   }

   /**
    * Deselects all the linear axes and clears the linear selection frame.
    * <p>
    * The angular part remains unmodified.
    * </p>
    */
   public void clearLinearSelection()
   {
      linearPart.clearSelection();
   }

   /**
    * Sets this selection matrix to only select the angular axes.
    * <p>
    * The selection frames are cleared and until the selection is changed, this selection matrix is
    * independent from its selection frame.
    * </p>
    */
   public void setToAngularSelectionOnly()
   {
      angularPart.resetSelection();
      linearPart.clearSelection();
   }

   /**
    * Sets this selection matrix to only select the linear axes.
    * <p>
    * The selection frames are cleared and until the selection is changed, this selection matrix is
    * independent from its selection frame.
    * </p>
    */
   public void setToLinearSelectionOnly()
   {
      angularPart.clearSelection();
      linearPart.resetSelection();
   }

   /**
    * Sets this selection matrix to {@code other}.
    * 
    * @param other the other selection matrix. Not modified.
    */
   public void set(SelectionMatrix6D other)
   {
      angularPart.set(other.angularPart);
      linearPart.set(other.linearPart);
   }

   /**
    * Sets the angular part of this selection matrix to {@code angularPart}.
    * 
    * @param angularPart the new value for the angular part of this selection matrix. Not modified.
    */
   public void setAngularPart(SelectionMatrix3D angularPart)
   {
      this.angularPart.set(angularPart);
   }

   /**
    * Sets the linear part of this selection matrix to {@code linearPart}.
    * 
    * @param linearPart the new value for the linear part of this selection matrix. Not modified.
    */
   public void setLinearPart(SelectionMatrix3D linearPart)
   {
      this.linearPart.set(linearPart);
   }

   /**
    * Updates the selection of the axes of interest for the angular part of this selection matrix.
    * <p>
    * Note that it is preferable to also set angular selection frame to which this selection is
    * referring to.
    * </p>
    * <p>
    * The linear part of this selection matrix remains unchanged.
    * </p>
    * 
    * @param xSelected whether the angular x-axis is an axis of interest.
    * @param ySelected whether the angular y-axis is an axis of interest.
    * @param zSelected whether the angular z-axis is an axis of interest.
    */
   public void setAngularAxisSelection(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      selectAngularX(xSelected);
      selectAngularY(ySelected);
      selectAngularZ(zSelected);
   }

   /**
    * Updates the selection state for the angular x-axis, does not change the state of the other
    * axes.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the angular x-axis is an axis of interest.
    */
   public void selectAngularX(boolean select)
   {
      angularPart.selectXAxis(select);
   }

   /**
    * Updates the selection state for the angular y-axis, does not change the state of the other
    * axes.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the angular y-axis is an axis of interest.
    */
   public void selectAngularY(boolean select)
   {
      angularPart.selectYAxis(select);
   }

   /**
    * Updates the selection state for the angular z-axis, does not change the state of the other
    * axes.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the angular z-axis is an axis of interest.
    */
   public void selectAngularZ(boolean select)
   {
      angularPart.selectZAxis(select);
   }

   /**
    * Updates the selection of the axes of interest for the linear part of this selection matrix.
    * <p>
    * Note that it is preferable to also set linear selection frame to which this selection is
    * referring to.
    * </p>
    * <p>
    * The angular part of this selection matrix remains unchanged.
    * </p>
    * 
    * @param xSelected whether the linear x-axis is an axis of interest.
    * @param ySelected whether the linear y-axis is an axis of interest.
    * @param zSelected whether the linear z-axis is an axis of interest.
    */
   public void setLinearAxisSelection(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      selectLinearX(xSelected);
      selectLinearY(ySelected);
      selectLinearZ(zSelected);
   }

   /**
    * Updates the selection state for the linear x-axis, does not change the state of the other
    * axes.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the linear x-axis is an axis of interest.
    */
   public void selectLinearX(boolean select)
   {
      linearPart.selectXAxis(select);
   }

   /**
    * Updates the selection state for the linear y-axis, does not change the state of the other
    * axes.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the linear y-axis is an axis of interest.
    */
   public void selectLinearY(boolean select)
   {
      linearPart.selectYAxis(select);
   }

   /**
    * Updates the selection state for the linear z-axis, does not change the state of the other
    * axes.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the linear z-axis is an axis of interest.
    */
   public void selectLinearZ(boolean select)
   {
      linearPart.selectZAxis(select);
   }

   /**
    * Selects an axis based on {@code axisIndex} and updates update the selection state to
    * {@code select}.
    * <p>
    * For an {@code axisIndex} value going from 0 up to 5, the corresponding components are
    * {@code angularX}, {@code angularY}, {@code angularZ}, {@code linearX}, {@code linearY}, and
    * {@code linearZ}, respectively.
    * </p>
    *
    * @param axisIndex the index of the axis to update the selection state of.
    * @param select whether the chosen axis is an axis of interest.
    * @throws IndexOutOfBoundsException if {@code axisIndex} &notin; [0, 5].
    */
   public void selectAxis(int axisIndex, boolean select)
   {
      switch (axisIndex)
      {
      case 0:
         selectAngularX(select);
         break;
      case 1:
         selectAngularY(select);
         break;
      case 2:
         selectAngularZ(select);
         break;
      case 3:
         selectLinearX(select);
         break;
      case 4:
         selectLinearY(select);
         break;
      case 5:
         selectLinearZ(select);
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(axisIndex));
      }
   }

   /**
    * Applies the angular part of this selection matrix on the given vector:<br>
    * v' = A<sub>3x3</sub> * v<br>
    * where v is the given vector, A<sub>3x3</sub> the angular part of this selection matrix, and v'
    * the result of the selection.
    * 
    * @param vectorToBeModified the vector on which the angular part of this selection matrix to be
    *           applied. Modified.
    */
   public void applyAngularSelection(FrameVector3D vectorToBeModified)
   {
      angularPart.applySelection(vectorToBeModified);
   }

   /**
    * Applies this selection matrix on the given vector:<br>
    * v' = L<sub>3x3</sub> * v<br>
    * where v is the given vector, L<sub>3x3</sub> the linear part of this selection matrix, and v'
    * the result of the selection.
    * 
    * @param vectorToBeModified the vector on which the linear part of this selection matrix to be
    *           applied. Modified.
    */
   public void applyLinearSelection(FrameVector3D vectorToBeModified)
   {
      linearPart.applySelection(vectorToBeModified);
   }

   /**
    * Converts this into an actual 6-by-6 selection matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * The given {@code selectionMatrixToPack} is first reshaped to be a 6-by-6 matrix which will be
    * set to represent this.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the selection matrix is to be used.
    * @param selectionMatrixToPack the dense-matrix is first reshaped to a 6-by-6 matrix and then
    *           set to represent this. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getFullSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrixToPack.reshape(6, 6);
      selectionMatrixToPack.zero();

      angularPart.getFullSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
      linearPart.getFullSelectionMatrixInFrame(destinationFrame, 3, 3, selectionMatrixToPack);
   }

   /**
    * Converts this into an actual 6-by-6 selection matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * In addition to what {@link #getFullSelectionMatrixInFrame(ReferenceFrame, DenseMatrix64F)}
    * does, this method also removes the zero-rows of the given selection matrix. This will help to
    * improve performance especially when the resulting selection matrix is to be multiplied to a
    * large matrix.
    * </p>
    * <p>
    * The given {@code selectionMatrixToPack} is first reshaped to be a 6-by-6 matrix which will be
    * set to represent this.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the selection matrix is to be used.
    * @param selectionMatrixToPack the dense-matrix is first reshaped to a 6-by-6 matrix and then
    *           set to represent this. The zero-rows of the resulting matrix are removed. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getCompactSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      selectionMatrixToPack.reshape(6, 6);
      selectionMatrixToPack.zero();

      // Need to do the linear part first as rows might be removed.
      linearPart.getCompactSelectionMatrixInFrame(destinationFrame, 3, 3, selectionMatrixToPack);
      angularPart.getCompactSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
   }

   /**
    * Whether the angular x-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the angular x-axis.
    */
   public boolean isAngularXSelected()
   {
      return angularPart.isXSelected();
   }

   /**
    * Whether the angular y-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the angular y-axis.
    */
   public boolean isAngularYSelected()
   {
      return angularPart.isYSelected();
   }

   /**
    * Whether the angular z-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the angular z-axis.
    */
   public boolean isAngularZSelected()
   {
      return angularPart.isZSelected();
   }

   /**
    * The reference frame to which the angular axis selection is referring to.
    * <p>
    * This selection frame can be {@code null}.
    * </p>
    * 
    * @return the current selection frame for the angular part of this selection matrix.
    */
   public ReferenceFrame getAngularSelectionFrame()
   {
      return angularPart.getSelectionFrame();
   }

   /**
    * Whether the linear x-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the linear x-axis.
    */
   public boolean isLinearXSelected()
   {
      return linearPart.isXSelected();
   }

   /**
    * Whether the linear y-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the linear y-axis.
    */
   public boolean isLinearYSelected()
   {
      return linearPart.isYSelected();
   }

   /**
    * Whether the linear z-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the linear z-axis.
    */
   public boolean isLinearZSelected()
   {
      return linearPart.isZSelected();
   }

   /**
    * Whether the {@code axisIndex}<sup>th</sup> axis of the current selection frame has been selected.
    * <p>
    * For an {@code axisIndex} value going from 0 up to 5, the corresponding components are
    * {@code angularX}, {@code angularY}, {@code angularZ}, {@code linearX}, {@code linearY}, and
    * {@code linearZ}, respectively.
    * </p>
    *
    * @param axisIndex the index of the axis to get the selection state of.
    * @return the selection state of the chosen axis.
    * @throws IndexOutOfBoundsException if {@code axisIndex} &notin; [0, 5].
    */
   public boolean isAxisSelected(int axisIndex)
   {
      switch (axisIndex)
      {
      case 0:
         return isAngularXSelected();
      case 1:
         return isAngularYSelected();
      case 2:
         return isAngularZSelected();
      case 3:
         return isLinearXSelected();
      case 4:
         return isLinearYSelected();
      case 5:
         return isLinearZSelected();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(axisIndex));
      }
   }

   /**
    * The reference frame to which the linear axis selection is referring to.
    * <p>
    * This selection frame can be {@code null}.
    * </p>
    * 
    * @return the current selection frame for the linear part of this selection matrix.
    */
   public ReferenceFrame getLinearSelectionFrame()
   {
      return linearPart.getSelectionFrame();
   }

   /**
    * Returns the internal reference to the angular part of this selection matrix.
    * 
    * @return the angular part.
    */
   public SelectionMatrix3D getAngularPart()
   {
      return angularPart;
   }

   /**
    * Returns the internal reference to the linear part of this selection matrix.
    * 
    * @return the linear part.
    */
   public SelectionMatrix3D getLinearPart()
   {
      return linearPart;
   }

   /**
    * Returns true if any of the angular axis are enabled
    * 
    * @return
    */
   public boolean isAngularPartActive()
   {
      return isAngularXSelected() || isAngularYSelected() || isAngularZSelected();
   }

   /**
    * Returns true if any of the linear axis are enabled
    * 
    * @return
    */
   public boolean isLinearPartActive()
   {
      return isLinearXSelected() || isLinearYSelected() || isLinearZSelected();
   }

   @Override
   public String toString()
   {
      return "Angular: " + angularPart + ", linear: " + linearPart;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((angularPart == null) ? 0 : angularPart.hashCode());
      result = prime * result + ((linearPart == null) ? 0 : linearPart.hashCode());
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      SelectionMatrix6D other = (SelectionMatrix6D) obj;
      if (angularPart == null)
      {
         if (other.angularPart != null)
            return false;
      }
      else if (!angularPart.equals(other.angularPart))
         return false;
      if (linearPart == null)
      {
         if (other.linearPart != null)
            return false;
      }
      else if (!linearPart.equals(other.linearPart))
         return false;
      return true;
   }
}

package us.ihmc.robotics.math.frames;

import org.apache.commons.lang3.StringUtils;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly since they contain YoVariables.
public abstract class YoFrameTuple<S, T extends FrameTuple3D<T, ?>> implements ReferenceFrameHolder, Clearable, FrameTuple3DReadOnly
{
   private final String namePrefix;
   private final String nameSuffix;

   private final YoDouble x, y, z; // This is where the data is stored. All operations must act on these numbers.
   private final T frameTuple; // This is only for assistance. The data is stored in the YoVariables, not in here!
   /** Never use this reference frame directly, use {@link #getReferenceFrame()} instead so the multiple frames version of this {@link YoFrameTuple} will work properly. */
   private final ReferenceFrame referenceFrame; // Redundant but allows to make sure the frame isn't changed

   public YoFrameTuple(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable, ReferenceFrame referenceFrame)
   {
      this.namePrefix = StringUtils.getCommonPrefix(xVariable.getName(), yVariable.getName(), zVariable.getName());
      this.nameSuffix = YoFrameVariableNameTools.getCommonSuffix(xVariable.getName(), yVariable.getName(), zVariable.getName());

      this.x = xVariable;
      this.y = yVariable;
      this.z = zVariable;
      this.referenceFrame = referenceFrame;
      this.frameTuple = createEmptyFrameTuple();
   }

   public YoFrameTuple(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      x = new YoDouble(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;
      this.frameTuple = createEmptyFrameTuple();
   }

   public final T getFrameTuple()
   {
      putYoValuesIntoFrameTuple();
      return frameTuple;
   }

   public final double getX()
   {
      return x.getDoubleValue();
   }

   public final double getY()
   {
      return y.getDoubleValue();
   }

   public final double getZ()
   {
      return z.getDoubleValue();
   }

   public final double get(Axis axis)
   {
      switch (axis)
      {
      case X:
         return getX();

      case Y:
         return getY();

      case Z:
         return getZ();

      default:
         throw new IndexOutOfBoundsException();
      }
   }

   public final YoDouble getYoX()
   {
      return x;
   }

   public final YoDouble getYoY()
   {
      return y;
   }

   public final YoDouble getYoZ()
   {
      return z;
   }

   public final void setX(double newX)
   {
      x.set(newX);
   }

   public final void setY(double newY)
   {
      y.set(newY);
   }

   public final void setZ(double newZ)
   {
      z.set(newZ);
   }

   public final void set(double newX, double newY, double newZ)
   {
      x.set(newX);
      y.set(newY);
      z.set(newZ);
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from its first row index.
    *
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void set(DenseMatrix64F tupleDenseMatrix)
   {
      frameTuple.setIncludingFrame(getReferenceFrame(), tupleDenseMatrix);
      getYoValuesFromFrameTuple();
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given column
    * vector starting to read from {@code startRow}.
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void set(int startRow, DenseMatrix64F tupleDenseMatrix)
   {
      frameTuple.setIncludingFrame(getReferenceFrame(), startRow, tupleDenseMatrix);
      getYoValuesFromFrameTuple();
   }

   /**
    * Sets this tuple's components {@code x}, {@code y}, {@code z} in order from the given matrix
    * starting to read from {@code startRow} at the column index {@code column}.
    *
    * @param startRow the first row index to start reading in the dense-matrix.
    * @param column the column index to read in the dense-matrix.
    * @param matrix the column vector containing the new values for this tuple's components. Not
    *           modified.
    */
   public final void set(int startRow, int column, DenseMatrix64F tupleDenseMatrix)
   {
      frameTuple.setIncludingFrame(getReferenceFrame(), startRow, column, tupleDenseMatrix);
      getYoValuesFromFrameTuple();
   }

   public final void set(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(x, y, z);
   }

   public final void setAndMatchFrame(FrameTuple3DReadOnly frameTuple)
   {
      this.frameTuple.setIncludingFrame(frameTuple);
      this.frameTuple.changeFrame(getReferenceFrame());
      getYoValuesFromFrameTuple(true);
   }

   /**
    * Sets this tuple to the location of the origin of passed in referenceFrame.
    */
   protected void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.frameTuple.setToZero(referenceFrame);
      this.frameTuple.changeFrame(getReferenceFrame());
      getYoValuesFromFrameTuple(true);
   }
   
   public final void set(FrameTuple3DReadOnly frameTuple)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.set(frameTuple);
      getYoValuesFromFrameTuple(true);
   }

   public final void set(Tuple2DReadOnly tuple2d)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.set(tuple2d);
      getYoValuesFromFrameTuple();
   }

   public final void set(Tuple2DReadOnly tuple2d, double z)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.set(tuple2d, z);
      getYoValuesFromFrameTuple();
   }

   public final void set(FrameTuple2DReadOnly frameTuple2d)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.set(frameTuple2d);
      getYoValuesFromFrameTuple();
   }

   public final void set(FrameTuple2DReadOnly frameTuple2d, double z)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.set(frameTuple2d, z);
      getYoValuesFromFrameTuple();
   }

   public final void set(Tuple3DReadOnly tuple)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.set(tuple);
      getYoValuesFromFrameTuple();
   }

   public final void add(double dx, double dy, double dz)
   {
      x.set(x.getDoubleValue() + dx);
      y.set(y.getDoubleValue() + dy);
      z.set(z.getDoubleValue() + dz);
   }

   public final void add(Tuple3DReadOnly tuple)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.add(tuple);
      getYoValuesFromFrameTuple();
   }

   public final void add(FrameTuple3DReadOnly frameTuple)
   {
      putYoValuesIntoFrameTuple();
      this.frameTuple.add(frameTuple);
      getYoValuesFromFrameTuple();
   }

   public final void add(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.add(tuple1, tuple2);
      getYoValuesFromFrameTuple();
   }

   public final void add(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.add(frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   public final void sub(Tuple3DReadOnly tuple)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.sub(tuple);
      getYoValuesFromFrameTuple();
   }

   public final void sub(FrameTuple3DReadOnly frameTuple)
   {
      putYoValuesIntoFrameTuple();
      this.frameTuple.sub(frameTuple);
      getYoValuesFromFrameTuple();
   }

   public final void sub(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      frameTuple.sub(tuple1, tuple2);
      getYoValuesFromFrameTuple();
   }

   public final void sub(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.sub(frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   public final void scale(double scaleFactor)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.scale(scaleFactor);
      getYoValuesFromFrameTuple();
   }

   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly frameTuple)
   {
      putYoValuesIntoFrameTuple();
      this.frameTuple.scaleAdd(scaleFactor, frameTuple);
      getYoValuesFromFrameTuple();
   }

   /**
    * Sets the value of this yoFrameTuple to the scalar multiplication of frameTuple1 and then adds frameTuple2 (this = scaleFactor * frameTuple1 + frameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?, ?>
    * @param frameTuple2 FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.scaleAdd(scaleFactor, frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   /**
    * Sets the value of this yoFrameTuple to the scalar multiplication of frameTuple1 and then subs frameTuple2 (this = scaleFactor * frameTuple1 - frameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple<?, ?>
    * @param frameTuple2 FrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleSub(double scaleFactor, FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.scaleSub(scaleFactor, frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   /**
    * Negates the value of this YoFrameTuple in place.
    */
   public final void negate()
   {
      set(-getX(), -getY(), -getZ());
   }

   /**
    *  Linearly interpolates between tuples frameTuple1 and frameTuple2 and places the result into this tuple:  this = (1-alpha) * frameTuple1 + alpha * frameTuple2.
    *  @param frameTuple1  the first tuple
    *  @param frameTuple2  the second tuple  
    *  @param alpha  the alpha interpolation parameter
    * @throws ReferenceFrameMismatchException
    */
   public final void interpolate(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      putYoValuesIntoFrameTuple();
      frameTuple.interpolate(frameTuple1, frameTuple2, alpha);
      getYoValuesFromFrameTuple();
   }

   @Override
   public boolean containsNaN()
   {
      putYoValuesIntoFrameTuple();
      return frameTuple.containsNaN();
   }

   @Override
   public final void setToZero()
   {
      frameTuple.setToZero(getReferenceFrame());
      getYoValuesFromFrameTuple(true);
   }

   @Override
   public final void setToNaN()
   {
      frameTuple.setToNaN(getReferenceFrame());
      getYoValuesFromFrameTuple();
   }

   public final void applyTransform(Transform transform)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.applyTransform(transform);
      getYoValuesFromFrameTuple();
   }

   /**
    * Selects a component of this tuple based on {@code index} and sets it to {@code value}.
    * <p>
    * For an {@code index} of 0, the corresponding component is {@code x}, 1 it is {@code y}, 2 it
    * is {@code z}.
    * </p>
    *
    * @param index the index of the component to set.
    * @param value the new value of the selected component.
    * @throws IndexOutOfBoundsException if {@code index} &notin; [0, 2].
    */
   public void setElement(int index, double value)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.setElement(index, value);
      getYoValuesFromFrameTuple();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void notifyVariableChangedListeners()
   {
      x.notifyVariableChangedListeners(); // No need to do it for all
   }

   public final void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      x.addVariableChangedListener(variableChangedListener);
      y.addVariableChangedListener(variableChangedListener);
      z.addVariableChangedListener(variableChangedListener);
   }

   protected abstract T createEmptyFrameTuple();

   protected void putYoValuesIntoFrameTuple()
   {
      frameTuple.setIncludingFrame(getReferenceFrame(), x.getDoubleValue(), y.getDoubleValue(), z.getDoubleValue());
   }

   protected void getYoValuesFromFrameTuple()
   {
      getYoValuesFromFrameTuple(true);
   }

   private void getYoValuesFromFrameTuple(boolean notifyListeners)
   {
      x.set(frameTuple.getX(), notifyListeners);
      y.set(frameTuple.getY(), notifyListeners);
      z.set(frameTuple.getZ(), notifyListeners);
   }

   /**
    * toString
    *
    * String representation of a FrameVector (x,y,z)-reference frame name
    *
    * @return String
    */
   @Override
   public String toString()
   {
      putYoValuesIntoFrameTuple();
      return frameTuple.toString();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   public String getNameSuffix()
   {
      return nameSuffix;
   }
}

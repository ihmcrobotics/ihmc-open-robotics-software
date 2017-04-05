package us.ihmc.robotics.math.frames;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameTuple2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly since they contain YoVariables.
public abstract class YoFrameTuple<S, T extends FrameTuple<?, ?>> extends AbstractReferenceFrameHolder implements Clearable
{
   private final String namePrefix;
   private final String nameSuffix;

   private final DoubleYoVariable x, y, z; // This is where the data is stored. All operations must act on these numbers.
   private final T frameTuple; // This is only for assistance. The data is stored in the YoVariables, not in here!
   /** Never use this reference frame directly, use {@link #getReferenceFrame()} instead so the multiple frames version of this {@link YoFrameTuple} will work properly. */
   private final ReferenceFrame referenceFrame; // Redundant but allows to make sure the frame isn't changed

   public YoFrameTuple(DoubleYoVariable xVariable, DoubleYoVariable yVariable, DoubleYoVariable zVariable, ReferenceFrame referenceFrame)
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

      x = new DoubleYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), registry);
      y = new DoubleYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), registry);
      z = new DoubleYoVariable(YoFrameVariableNameTools.createZName(namePrefix, nameSuffix), registry);
      this.referenceFrame = referenceFrame;
      this.frameTuple = createEmptyFrameTuple();
   }

   public final void get(Tuple3DBasics tuple3dToPack)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.get(tuple3dToPack);
   }

   public final Vector3D getVector3dCopy()
   {
      Vector3D vector3d = new Vector3D();
      get(vector3d);
      return vector3d;
   }

   public final Point3D getPoint3dCopy()
   {
      Point3D point3d = new Point3D();
      get(point3d);
      return point3d;
   }

   public final void getFrameTuple(FrameTuple<?, ?> frameTuple)
   {
      frameTuple.set(getFrameTuple());
   }

   public final void getFrameTuple2d(FrameTuple2d<?, ?> frameTuple2d)
   {
      frameTuple2d.setByProjectionOntoXYPlane(getFrameTuple());
   }

   public final T getFrameTuple()
   {
      putYoValuesIntoFrameTuple();
      return frameTuple;
   }

   public final FrameVector getFrameVectorCopy()
   {
      return new FrameVector(getFrameTuple());
   }

   public final FramePoint getFramePointCopy()
   {
      return new FramePoint(getFrameTuple());
   }

   public final FrameVector2d getFrameVector2dCopy()
   {
      return new FrameVector2d(getReferenceFrame(), getX(), getY());
   }

   public final FramePoint2d getFramePoint2dCopy()
   {
      return new FramePoint2d(getReferenceFrame(), getX(), getY());
   }

   public final void getFrameTupleIncludingFrame(FrameTuple<?, ?> frameTuple)
   {
      frameTuple.setIncludingFrame(getReferenceFrame(), getX(), getY(), getZ());
   }

   public final void getFrameTuple2dIncludingFrame(FrameTuple2d<?, ?> frameTuple2d)
   {
      putYoValuesIntoFrameTuple();
      frameTuple2d.setIncludingFrame(getReferenceFrame(), getX(), getY());
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

   public final double get(Direction direction)
   {
      switch (direction)
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

   public final DoubleYoVariable getYoX()
   {
      return x;
   }

   public final DoubleYoVariable getYoY()
   {
      return y;
   }

   public final DoubleYoVariable getYoZ()
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
    * Sets x, y, and z with no checks for reference frame matches.
    * @deprecated the user should simply use {@link #set(Tuple3DBasics)} instead.
    */
   @Deprecated
   public final void setWithoutChecks(FrameTuple<?, ?> frameTuple)
   {
      x.set(frameTuple.getX());
      y.set(frameTuple.getY());
      z.set(frameTuple.getZ());
   }

   public final void set(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(x, y, z);
   }

   public final void setAndMatchFrame(FrameTuple<?, ?> frameTuple)
   {
      setAndMatchFrame(frameTuple, true);
   }

   public final void setAndMatchFrame(FrameTuple<?, ?> frameTuple, boolean notifyListeners)
   {
      this.frameTuple.setIncludingFrame(frameTuple);
      this.frameTuple.changeFrame(getReferenceFrame());
      getYoValuesFromFrameTuple(notifyListeners);
   }

   public final void setAndMatchFrame(YoFrameTuple<?, ?> yoFrameTuple)
   {
      setAndMatchFrame(yoFrameTuple, true);
   }

   public final void setAndMatchFrame(YoFrameTuple<?, ?> yoFrameTuple, boolean notifyListeners)
   {
      yoFrameTuple.getFrameTupleIncludingFrame(frameTuple);
      this.frameTuple.changeFrame(getReferenceFrame());
      getYoValuesFromFrameTuple(notifyListeners);
   }
   
   /**
    * Sets this tuple to the location of the origin of passed in referenceFrame.
    */
   private final void setFromReferenceFrame(ReferenceFrame referenceFrame, boolean notifyListeners)
   {
      this.frameTuple.setToZero(referenceFrame);
      this.frameTuple.changeFrame(getReferenceFrame());
      getYoValuesFromFrameTuple(notifyListeners);
   }
   
   /**
    * Sets this tuple to the location of the origin of passed in referenceFrame.
    */
   protected void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setFromReferenceFrame(referenceFrame, true);
   }

   public final void set(FrameTuple<?, ?> frameTuple)
   {
      set(frameTuple, true);
   }

   public final void set(FrameTuple<?, ?> frameTuple, boolean notifyListeners)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.set(frameTuple);
      getYoValuesFromFrameTuple(notifyListeners);
   }

   public final void set(YoFrameTuple<?, ?> yoFrameTuple)
   {
      set(yoFrameTuple.getFrameTuple());
   }

   public final void setXY(Tuple2DReadOnly tuple2d)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.setXY(tuple2d);
      getYoValuesFromFrameTuple();
   }

   public final void setXY(FrameTuple2d<?, ?> frameTuple2d)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      this.frameTuple.setXY(frameTuple2d);
      getYoValuesFromFrameTuple();
   }

   public final void setXY(YoFrameTuple2d<?, ?> yoFrameTuple2d)
   {
      this.frameTuple.setToZero(getReferenceFrame());
      yoFrameTuple2d.getFrameTupleIncludingFrame(frameTuple);
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

   public final void add(FrameTuple<?, ?> frameTuple)
   {
      putYoValuesIntoFrameTuple();
      this.frameTuple.add(frameTuple);
      getYoValuesFromFrameTuple();
   }

   public final void add(YoFrameTuple<?, ?> yoFrameTuple)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.add(yoFrameTuple.getFrameTuple());
      getYoValuesFromFrameTuple();
   }

   public final void add(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.add(tuple1, tuple2);
      getYoValuesFromFrameTuple();
   }

   public final void add(FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.add(frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   public final void add(YoFrameTuple<?, ?> yoFrameTuple1, YoFrameTuple<?, ?> yoFrameTuple2)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.add(yoFrameTuple1.getFrameTuple(), yoFrameTuple2.getFrameTuple());
      getYoValuesFromFrameTuple();
   }

   public final void sub(Tuple3DReadOnly tuple)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.sub(tuple);
      getYoValuesFromFrameTuple();
   }

   public final void sub(FrameTuple<?, ?> frameTuple)
   {
      putYoValuesIntoFrameTuple();
      this.frameTuple.sub(frameTuple);
      getYoValuesFromFrameTuple();
   }

   public final void sub(YoFrameTuple<?, ?> yoFrameTuple)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.sub(yoFrameTuple.getFrameTuple());
      getYoValuesFromFrameTuple();
   }

   public final void sub(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      frameTuple.sub(tuple1, tuple2);
      getYoValuesFromFrameTuple();
   }

   public final void sub(FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.sub(frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   public final void sub(YoFrameTuple<?, ?> yoFrameTuple1, YoFrameTuple<?, ?> yoFrameTuple2)
   {
      sub(yoFrameTuple1.getFrameTuple(), yoFrameTuple2.getFrameTuple());
   }

   public final void scale(double scaleFactor)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.scale(scaleFactor);
      getYoValuesFromFrameTuple();
   }

   public final void scaleAdd(double scaleFactor, FrameTuple<?, ?> frameTuple)
   {
      putYoValuesIntoFrameTuple();
      this.frameTuple.scaleAdd(scaleFactor, frameTuple);
      getYoValuesFromFrameTuple();
   }

   public final void scaleAdd(double scaleFactor, YoFrameTuple<?, ?> yoFrameTuple)
   {
      scaleAdd(scaleFactor, yoFrameTuple.getFrameTuple());
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
   public final void scaleAdd(double scaleFactor, FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.scaleAdd(scaleFactor, frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   /**
    * Sets the value of this yoFrameTuple to the scalar multiplication of yoFrameTuple1 and then adds yoFrameTuple2 (this = scaleFactor * yoFrameTuple1 + yoFrameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param yoFrameTuple1 YoFrameTuple<?, ?>
    * @param yoFrameTuple2 YoFrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, YoFrameTuple<?, ?> yoFrameTuple1, YoFrameTuple<?, ?> yoFrameTuple2)
   {
      scaleAdd(scaleFactor, yoFrameTuple1.getFrameTuple(), yoFrameTuple2.getFrameTuple());
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
   public final void scaleSub(double scaleFactor, FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      frameTuple.setToZero(getReferenceFrame());
      frameTuple.scaleSub(scaleFactor, frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   /**
    * Sets the value of this yoFrameTuple to the scalar multiplication of yoFrameTuple1 and then subs yoFrameTuple2 (this = scaleFactor * yoFrameTuple1 - yoFrameTuple2).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param yoFrameTuple1 YoFrameTuple<?, ?>
    * @param yoFrameTuple2 YoFrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleSub(double scaleFactor, YoFrameTuple<?, ?> yoFrameTuple1, YoFrameTuple<?, ?> yoFrameTuple2)
   {
      scaleSub(scaleFactor, yoFrameTuple1.getFrameTuple(), yoFrameTuple2.getFrameTuple());
   }

   /**
    * Sets the value of this yoFrameTuple to the scalar multiplication of yoFrameTuple1 minus yoFrameTuple2 (this = scaleFactor * ( yoFrameTuple1 - yoFrameTuple2 ) ).
    *
    * @param scaleFactor double
    * @param frameTuple1 the first YoFrameTuple<?, ?>
    * @param frameTuple2 the second YoFrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void subAndScale(double scaleFactor, FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.subAndScale(scaleFactor, frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple();
   }

   /**
    * Sets the value of this yoFrameTuple to the scalar multiplication of yoFrameTuple1 minus yoFrameTuple2 (this = scaleFactor * ( yoFrameTuple1 - yoFrameTuple2 ) ).
    *
    * @param scaleFactor double
    * @param yoFrameTuple1 the first YoFrameTuple<?, ?>
    * @param yoFrameTuple2 the second YoFrameTuple<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void subAndScale(double scaleFactor, YoFrameTuple<?, ?> yoFrameTuple1, YoFrameTuple<?, ?> yoFrameTuple2)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.subAndScale(scaleFactor, yoFrameTuple1.getFrameTuple(), yoFrameTuple2.getFrameTuple());
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
   public final void interpolate(FrameTuple<?, ?> frameTuple1, FrameTuple<?, ?> frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      putYoValuesIntoFrameTuple();
      frameTuple.interpolate(frameTuple1, frameTuple2, alpha);
      getYoValuesFromFrameTuple();
   }

   /**
    *  Linearly interpolates between tuples yoFrameTuple1 and yoFrameTuple2 and places the result into this tuple:  this = (1-alpha) * yoFrameTuple1 + alpha * yoFrameTuple2.
    *  @param yoFrameTuple1  the first tuple
    *  @param yoFrameTuple2  the second tuple  
    *  @param alpha  the alpha interpolation parameter
    * @throws ReferenceFrameMismatchException
    */
   public final void interpolate(YoFrameTuple<?, ?> yoFrameTuple1, YoFrameTuple<?, ?> yoFrameTuple2, double alpha)
   {
      checkReferenceFrameMatch(yoFrameTuple1);
      checkReferenceFrameMatch(yoFrameTuple2);
      interpolate(yoFrameTuple1.getFrameTuple(), yoFrameTuple2.getFrameTuple(), alpha);
   }

   public final boolean epsilonEquals(YoFrameTuple<?, ?> frameTuple, double threshold)
   {
      putYoValuesIntoFrameTuple();
      return this.frameTuple.epsilonEquals(frameTuple.getFrameTuple(), threshold);
   }

   public final boolean epsilonEquals(FrameTuple<?, ?> frameTuple, double threshold)
   {
      putYoValuesIntoFrameTuple();
      return this.frameTuple.epsilonEquals(frameTuple, threshold);
   }

   public final boolean epsilonEquals(FrameTuple2d<?, ?> frameTuple2d, double threshold)
   {
      putYoValuesIntoFrameTuple();
      return this.frameTuple.epsilonEquals(frameTuple2d, threshold);
   }

   public final void checkForNaN()
   {
      putYoValuesIntoFrameTuple();
      frameTuple.checkForNaN();
   }

   @Override
   public final boolean containsNaN()
   {
      putYoValuesIntoFrameTuple();
      return frameTuple.containsNaN();
   }

   @Override
   public final void setToZero()
   {
      setToZero(false);
   }

   public final void setToZero(boolean notifyListeners)
   {
      frameTuple.setToZero(getReferenceFrame());
      getYoValuesFromFrameTuple(notifyListeners);
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

   public final void set(Direction direction, double value)
   {
      putYoValuesIntoFrameTuple();
      frameTuple.set(direction, value);
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

package us.ihmc.robotics.math.frames;

import javax.vecmath.Point2d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly since they contain YoVariables.
public abstract class YoFrameTuple2d<S, T extends FrameTuple2d<?, ?>> extends AbstractReferenceFrameHolder
{
   private final DoubleYoVariable x, y; // This is where the data is stored. All operations must act on these numbers.
   private final T frameTuple2d; // This is only for assistance. The data is stored in the YoVariables, not in here!
   private final ReferenceFrame referenceFrame; // Redundant but allows to make sure the frame isn't changed


   public YoFrameTuple2d(DoubleYoVariable xVariable, DoubleYoVariable yVariable, ReferenceFrame referenceFrame)
   {
      this.x = xVariable;
      this.y = yVariable;
      this.referenceFrame = referenceFrame;
      this.frameTuple2d = createEmptyFrameTuple2d();
      putYoValuesIntoFrameTuple2d();
   }

   public YoFrameTuple2d(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, "", referenceFrame, registry);
   }

   public YoFrameTuple2d(String namePrefix, String nameSuffix, String description, ReferenceFrame referenceFrame, YoVariableRegistry registry)
   {
      x = new DoubleYoVariable(YoFrameVariableNameTools.createXName(namePrefix, nameSuffix), description, registry);
      y = new DoubleYoVariable(YoFrameVariableNameTools.createYName(namePrefix, nameSuffix), description, registry);
      this.referenceFrame = referenceFrame;
      this.frameTuple2d = createEmptyFrameTuple2d();
      putYoValuesIntoFrameTuple2d();
   }

   public final void get(Tuple2d tuple2dToPack)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.get(tuple2dToPack);
   }

   /**
    * Pack this tuple2d in tuple3dToPack and tuple3dToPack.z = 0.0.
    * @param tuple3dToPack {@code Tuple3d}
    */
   public final void get(Tuple3d tuple3dToPack)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.get(tuple3dToPack);
   }
   
   /**
    * Pack this tuple2d in tuple3fToPack and tuple3fToPack.z = 0.0.
    * @param tuple3fToPack {@code Tuple3f}
    */
   public final void get(Tuple3f tuple3fToPack)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.get(tuple3fToPack);
   }

   public final Vector2d getVector2dCopy()
   {
      Vector2d vector2d = new Vector2d();
      get(vector2d);
      return vector2d;
   }

   public final Point2d getPoint2dCopy()
   {
      Point2d point2d = new Point2d();
      get(point2d);
      return point2d;
   }

   public final void getFrameTuple2d(FrameTuple2d<?, ?> frameTuple2dToPack)
   {
      frameTuple2dToPack.set(getFrameTuple2d());
   }

   public final T getFrameTuple2d()
   {
      putYoValuesIntoFrameTuple2d();
      return frameTuple2d;
   }

   public final FrameVector2d getFrameVector2dCopy()
   {
      return new FrameVector2d(getFrameTuple2d());
   }

   public final FramePoint2d getFramePoint2dCopy()
   {
      return new FramePoint2d(getFrameTuple2d());
   }
   
   public final void getFrameTuple2dIncludingFrame(FrameTuple2d<?, ?> frameTuple2dToPack)
   {
      frameTuple2dToPack.setIncludingFrame(getFrameTuple2d());
   }

   public final void getFrameTupleIncludingFrame(FrameTuple<?, ?> frameTupleToPack)
   {
      frameTupleToPack.setXYIncludingFrame(getFrameTuple2d());
   }

   public final double getX()
   {
      return x.getDoubleValue();
   }

   public final double getY()
   {
      return y.getDoubleValue();
   }

   public final DoubleYoVariable getYoX()
   {
      return x;
   }

   public final DoubleYoVariable getYoY()
   {
      return y;
   }

   public final void setX(double newX)
   {
      x.set(newX);
   }

   public final void setY(double newY)
   {
      y.set(newY);
   }

   public final void set(double newX, double newY)
   {
      x.set(newX);
      y.set(newY);
   }
   
   /**
    * Sets x and y with no checks for reference frame matches.
    */
   public final void setWithoutChecks(FrameTuple2d<?, ?> frameTuple2d)
   {
      x.set(frameTuple2d.getX());
      y.set(frameTuple2d.getY());
   }

   public final void setAndMatchFrame(FrameTuple2d<?, ?> frameTuple2d)
   {
      setAndMatchFrame(frameTuple2d, true);
   }

   public final void setAndMatchFrame(FrameTuple2d<?, ?> frameTuple2d, boolean notifyListeners)
   {
      this.frameTuple2d.setIncludingFrame(frameTuple2d);
      this.frameTuple2d.changeFrame(getReferenceFrame());
      getYoValuesFromFrameTuple2d(notifyListeners);
   }

   public final void set(ReferenceFrame referenceFrame, double x, double y)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(x, y);
   }

   public final void set(FrameTuple2d<?, ?> frameTuple2d)
   {
      set(frameTuple2d, true);
   }

   public final void set(FrameTuple2d<?, ?> frameTuple2d, boolean notifyListeners)
   {
      this.frameTuple2d.set(frameTuple2d);
      getYoValuesFromFrameTuple2d(notifyListeners);
   }

   public final void set(YoFrameTuple2d<?, ?> yoFrameTuple2d)
   {
      set(yoFrameTuple2d.getFrameTuple2d());
   }

   public final void set(Tuple2d tuple2d)
   {
      this.frameTuple2d.set(tuple2d);
      getYoValuesFromFrameTuple2d();
   }

   public final void setByProjectionOntoXYPlane(FrameTuple<?, ?> frameTuple)
   {
      setByProjectionOntoXYPlane(frameTuple, true);
   }

   public final void setByProjectionOntoXYPlane(FrameTuple<?, ?> frameTuple, boolean notifyListeners)
   {
      this.frameTuple2d.setByProjectionOntoXYPlane(frameTuple);
      getYoValuesFromFrameTuple2d(notifyListeners);
   }

   public final void setByProjectionOntoXYPlane(YoFrameTuple<?, ?> yoFrameTuple)
   {
      setByProjectionOntoXYPlane(yoFrameTuple, true);
   }

   public final void setByProjectionOntoXYPlane(YoFrameTuple<?, ?> yoFrameTuple, boolean notifyListeners)
   {
      yoFrameTuple.getFrameTuple2d(frameTuple2d);
      getYoValuesFromFrameTuple2d(notifyListeners);
   }

   public final void add(double dx, double dy)
   {
      x.set(x.getDoubleValue() + dx);
      y.set(y.getDoubleValue() + dy);
   }

   public final void add(Tuple2d tuple2d)
   {
      putYoValuesIntoFrameTuple2d();
      this.frameTuple2d.add(tuple2d);
      getYoValuesFromFrameTuple2d();
   }

   public final void add(FrameTuple2d<?, ?> frameTuple2d)
   {
      putYoValuesIntoFrameTuple2d();
      this.frameTuple2d.add(frameTuple2d);
      getYoValuesFromFrameTuple2d();
   }

   public final void add(YoFrameTuple2d<?, ?> yoFrameTuple2d)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.add(yoFrameTuple2d.getFrameTuple2d());
      getYoValuesFromFrameTuple2d();
   }

   public final void sub(Tuple2d tuple2d)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.sub(tuple2d);
      getYoValuesFromFrameTuple2d();
   }

   public final void sub(FrameTuple2d<?, ?> frameTuple2d)
   {
      putYoValuesIntoFrameTuple2d();
      this.frameTuple2d.sub(frameTuple2d);
      getYoValuesFromFrameTuple2d();
   }

   public final void sub(YoFrameTuple2d<?, ?> yoFrameTuple2d)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.sub(yoFrameTuple2d.getFrameTuple2d());
      getYoValuesFromFrameTuple2d();
   }

   public final void sub(FrameTuple2d<?, ?> frameTuple1, FrameTuple2d<?, ?> frameTuple2)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.sub(frameTuple1, frameTuple2);
      getYoValuesFromFrameTuple2d();
   }

   public final void scale(double scaleFactor)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.scale(scaleFactor);
      getYoValuesFromFrameTuple2d();
   }

   /**
    * Sets the value of this tuple to the scalar multiplication of itself and then adds frameTuple1 (this = scaleFactor * this + frameTuple1).
    * Checks if reference frames match.
    *
    * @param scaleFactor double
    * @param frameTuple1 FrameTuple2d<?, ?>
    * @throws ReferenceFrameMismatchException
    */
   public final void scaleAdd(double scaleFactor, FrameTuple2d<?, ?> frameTuple2d)
   {
      putYoValuesIntoFrameTuple2d();
      this.frameTuple2d.scaleAdd(scaleFactor, frameTuple2d);
      getYoValuesFromFrameTuple2d();
   }

   public final void scaleAdd(double scaleFactor, YoFrameTuple2d<?, ?> yoFrameTuple2d)
   {
      scaleAdd(scaleFactor, yoFrameTuple2d.getFrameTuple2d());
   }

   public final void scaleAdd(double scaleFactor, YoFrameTuple2d<?, ?> yoFrameTuple1, YoFrameTuple2d<?, ?> yoFrameTuple2)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.scaleAdd(scaleFactor, yoFrameTuple1.getFrameTuple2d(), yoFrameTuple2.getFrameTuple2d());
      getYoValuesFromFrameTuple2d();
   }

   public final void interpolate(Tuple2d tuple1, Tuple2d tuple2, double alpha)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.interpolate(tuple1, tuple2, alpha);
      getYoValuesFromFrameTuple2d();
   }

   /**
    *  Linearly interpolates between tuples frameTuple1 and frameTuple2 and places the result into this tuple:  this = (1-alpha) * frameTuple1 + alpha * frameTuple2.
    *  @param frameTuple1  the first tuple
    *  @param frameTuple2  the second tuple  
    *  @param alpha  the alpha interpolation parameter
    * @throws ReferenceFrameMismatchException
    */
   public final void interpolate(FrameTuple2d<?, ?> frameTuple1, FrameTuple2d<?, ?> frameTuple2, double alpha)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.interpolate(frameTuple1, frameTuple2, alpha);
      getYoValuesFromFrameTuple2d();
   }

   public final boolean epsilonEquals(FrameTuple2d<?, ?> frameTuple2d, double threshold)
   {
      putYoValuesIntoFrameTuple2d();
      return this.frameTuple2d.epsilonEquals(frameTuple2d, threshold);
   }

   public final void checkForNaN()
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.checkForNaN();
   }

   public final boolean containsNaN()
   {
      putYoValuesIntoFrameTuple2d();
      return frameTuple2d.containsNaN();
   }

   public final void setToZero()
   {
      frameTuple2d.setToZero(referenceFrame);
      getYoValuesFromFrameTuple2d();
   }

   public final void setToNaN()
   {
      frameTuple2d.setToNaN(referenceFrame);
      getYoValuesFromFrameTuple2d();
   }

   public final void applyTransform(RigidBodyTransform transform)
   {
      putYoValuesIntoFrameTuple2d();
      frameTuple2d.applyTransform(transform);
      getYoValuesFromFrameTuple2d();
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public final void attachVariableChangedListener(VariableChangedListener variableChangedListener)
   {
      x.addVariableChangedListener(variableChangedListener);
      y.addVariableChangedListener(variableChangedListener);
   }

   protected abstract T createEmptyFrameTuple2d();

   private final void putYoValuesIntoFrameTuple2d()
   {
      frameTuple2d.setIncludingFrame(referenceFrame, x.getDoubleValue(), y.getDoubleValue());
   }

   protected void getYoValuesFromFrameTuple2d()
   {
      getYoValuesFromFrameTuple2d(true);
   }

   private void getYoValuesFromFrameTuple2d(boolean notifyListeners)
   {
      x.set(frameTuple2d.getX(), notifyListeners);
      y.set(frameTuple2d.getY(), notifyListeners);
   }

   /**
    * toString
    *
    * String representation of a FrameVector (x,y)-reference frame name
    *
    * @return String
    */
   @Override
   public String toString()
   {
      putYoValuesIntoFrameTuple2d();
      return frameTuple2d.toString();
   }
}

package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.ReferenceFrameHolder;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class YoFrameConvexPolygon2d extends ReferenceFrameHolder implements VariableChangedListener
{
   private final ArrayList<YoFramePoint2d> yoFramePoints = new ArrayList<YoFramePoint2d>();
   private final IntegerYoVariable numVertices;
   private final ReferenceFrame referenceFrame;
   private final FrameConvexPolygon2d convexPolygon2dForReading;
   private final FrameConvexPolygon2d convexPolygon2dForWriting;

   private AtomicBoolean hasChanged = new AtomicBoolean(true);

   public YoFrameConvexPolygon2d(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, int maxNumberOfVertices, YoVariableRegistry registry)
   {
      this.numVertices = new IntegerYoVariable(namePrefix + "NumVertices" + nameSuffix, registry);
      numVertices.addVariableChangedListener(this);

      this.referenceFrame = referenceFrame;

      for (int i = 0; i < maxNumberOfVertices; i++)
      {
         YoFramePoint2d point = new YoFramePoint2d(namePrefix + "_" + i + "_", nameSuffix, referenceFrame, registry);
         point.attachVariableChangedListener(this);
         yoFramePoints.add(point);
      }

      convexPolygon2dForReading = new FrameConvexPolygon2d(referenceFrame);
      convexPolygon2dForWriting = new FrameConvexPolygon2d(referenceFrame);
   }

   public YoFrameConvexPolygon2d(ArrayList<YoFramePoint2d> yoFramePoints, IntegerYoVariable yoNumVertices, ReferenceFrame referenceFrame)
   {
      this.numVertices = yoNumVertices;
      numVertices.addVariableChangedListener(this);

      this.referenceFrame = referenceFrame;

      for (YoFramePoint2d point : yoFramePoints)
      {
         point.attachVariableChangedListener(this);
         this.yoFramePoints.add(point);
      }

      convexPolygon2dForReading = new FrameConvexPolygon2d(referenceFrame);
      convexPolygon2dForWriting = new FrameConvexPolygon2d(referenceFrame);
   }

   public void setFrameConvexPolygon2d(FrameConvexPolygon2d polygon)
   {
      if (polygon == null)
      {
         hide();
         setToNaN();
         return;
      }

      try
      {
         polygon.checkReferenceFrameMatch(referenceFrame);
         convexPolygon2dForWriting.setAndUpdate(polygon);
         getYoValuesFromFrameConvexPolygon2d();
      }
      catch (Exception e)
      {
         System.err.println("In YoFrameConvexPolygon2d.java: " + e.getClass().getSimpleName() + " while calling setConvexPolygon2d(FrameConvexPolygon2d).");
      }
   }

   public void setConvexPolygon2d(ConvexPolygon2d polygon)
   {
      if (polygon == null)
      {
         hide();
         setToNaN();
         return;
      }

      try
      {
         convexPolygon2dForWriting.clear(referenceFrame);
         convexPolygon2dForWriting.setAndUpdate(polygon);
         getYoValuesFromFrameConvexPolygon2d();
      }
      catch (Exception e)
      {
         System.err.println("In YoFrameConvexPolygon2d.java: " + e.getClass().getSimpleName() + " while calling setConvexPolygon2d(ConvexPolygon2d).");
      }
   }

   private void setToNaN()
   {
      for (int i = 0; i < yoFramePoints.size(); i++)
      {
         yoFramePoints.get(i).setToNaN();
      }
   }

   public int getNumberOfVertices()
   {
      return numVertices.getIntegerValue();
   }

   public IntegerYoVariable getYoNumberVertices()
   {
      return numVertices;
   }
   
   public FramePoint2d getFrameVertex(int vertexIndex)
   {
      checkIndexInBoundaries(vertexIndex);
      return yoFramePoints.get(vertexIndex).getFrameTuple2d();
   }

   public void checkIndexInBoundaries(int vertexIndex)
   {
      if (vertexIndex < 0)
         throw new IndexOutOfBoundsException("vertexIndex < 0");
      if (vertexIndex >= numVertices.getIntegerValue())
         throw new IndexOutOfBoundsException("vertexIndex >= numberOfVertices");
   }

   public int getMaxNumberOfVertices()
   {
      return yoFramePoints.size();
   }

   public FrameConvexPolygon2d getFrameConvexPolygon2d()
   {
      putYoValuesIntoFrameConvexPolygon2d();
      return this.convexPolygon2dForReading;
   }
   
   public void getFrameConvexPolygon2d(FrameConvexPolygon2d polygonToPack)
   {
      putYoValuesIntoFrameConvexPolygon2d();
      polygonToPack.setAndUpdate(convexPolygon2dForReading);
   }

   public ConvexPolygon2d getConvexPolygon2d()
   {
      putYoValuesIntoFrameConvexPolygon2d();
      return this.convexPolygon2dForReading.getConvexPolygon2d();
   }

   public ArrayList<YoFramePoint2d> getYoFramePoints()
   {
      return yoFramePoints;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   private void putYoValuesIntoFrameConvexPolygon2d()
   {
      try
      {
         convexPolygon2dForReading.clear(referenceFrame);
         for (int i = 0; i < numVertices.getIntegerValue(); i++)
            convexPolygon2dForReading.addVertex(yoFramePoints.get(i).getFrameTuple2d());
         convexPolygon2dForReading.update();
      }
      catch (Exception e)
      {
         System.err.println("In YoFrameConvexPolygon2d.java: " + e.getClass().getSimpleName() + " while calling putYoValuesIntoFrameConvexPolygon2d().");
      }
   }
   
   private void getYoValuesFromFrameConvexPolygon2d()
   {
      numVertices.set(convexPolygon2dForWriting.getNumberOfVertices());

      try
      {
         for (int i = 0; i < numVertices.getIntegerValue(); i++)
         {
            yoFramePoints.get(i).set(convexPolygon2dForWriting.getFrameVertex(i));
         }
      }
      catch (Exception e)
      {
         System.err.println("In YoFrameConvexPolygon2d.java: " + e.getClass().getSimpleName() + " while calling getYoValuesFromFrameConvexPolygon2d().");
         e.printStackTrace();
      }
   }
   
   public void hide()
   {
      numVertices.set(-1);
   }

   public boolean getHasChangedAndReset()
   {
      return hasChanged.getAndSet(false);
   }

   public void variableChanged(YoVariable<?> v)
   {
      hasChanged.set(true);
   }
}

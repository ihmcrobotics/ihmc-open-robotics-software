package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class YoPlaneContactState implements PlaneContactState, ModifiableContactState
{
   private final String namePrefix;
   private final YoVariableRegistry registry;
   private final ReferenceFrame frameAfterJoint;
   private final ReferenceFrame planeFrame;
   private final List<YoFramePoint2d> contactPoints = new ArrayList<YoFramePoint2d>();
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final YoFrameVector contactNormalFrameVector;

   public YoPlaneContactState(String namePrefix, ReferenceFrame frameAfterJoint, ReferenceFrame planeFrame, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.frameAfterJoint = frameAfterJoint;
      this.planeFrame = planeFrame;
      parentRegistry.addChild(registry);
      this.contactNormalFrameVector = new YoFrameVector(namePrefix + "ContactNormalFrameVector", planeFrame, registry);
   }

   public void set(List<FramePoint2d> contactPoints, double coefficientOfFriction, FrameVector normalContactVector)
   {
      this.contactNormalFrameVector.set(normalContactVector);

      set(contactPoints, coefficientOfFriction);
   }

   public void set(List<FramePoint2d> contactPoints, double coefficientOfFriction)
   {
      createYoFramePoints(contactPoints);

      FramePoint2d temp = new FramePoint2d(planeFrame);
      inContact.set(false);

      for (int i = 0; i < this.contactPoints.size(); i++)
      {
         YoFramePoint2d contactPoint = this.contactPoints.get(i);
         if (i < contactPoints.size())
         {
            FramePoint2d point = contactPoints.get(i);
            temp.setAndChangeFrame(point);
            temp.changeFrame(planeFrame);
            contactPoint.set(temp);
            inContact.set(true);
         }
         else
            invalidateContactPoint(contactPoint);
      }

      if (coefficientOfFriction < 0.0)
         throw new RuntimeException("Coefficient of friction is negative: " + coefficientOfFriction);
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   private void invalidateContactPoint(YoFramePoint2d contactPoint)
   {
      contactPoint.set(Double.NaN, Double.NaN);
   }

   public List<FramePoint2d> getContactPoints2d()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(contactPoints.size());
      for (YoFramePoint2d contactPoint : contactPoints)
      {
         if (!contactPoint.containsNaN())
         {
            ret.add(contactPoint.getFramePoint2dCopy());
         }
      }

      return ret;
   }

   public ReferenceFrame getBodyFrame()
   {
      return frameAfterJoint;
   }

   private void createYoFramePoints(List<? extends FramePoint2d> contactPoints)
   {
      int oldSize = this.contactPoints.size();
      int newSize = contactPoints.size();
      for (int i = oldSize; i < newSize; i++)
      {
         this.contactPoints.add(new YoFramePoint2d(namePrefix + "Contact" + i, "", planeFrame, registry));
      }
   }

   public List<FramePoint> getContactPoints()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactPoints.size());
      for (YoFramePoint2d contactPoint : contactPoints)
      {
         if (!contactPoint.containsNaN())
         {
            ret.add(new FramePoint(contactPoint.getReferenceFrame(), contactPoint.getX(), contactPoint.getY(), 0.0));
         }
      }

      return ret;
   }

   public ReferenceFrame getPlaneFrame()
   {
      return planeFrame;
   }

   public boolean inContact()
   {
      return inContact.getBooleanValue();
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction.getDoubleValue();
   }

   public int getNumberOfContactPoints()
   {
      int ret = 0;
      for (YoFramePoint2d contactPoint : contactPoints)
      {
         if (!contactPoint.containsNaN())
            ret++;
      }
      return ret;
   }

   public FrameVector getContactNormalFrameVector()
   {
      return contactNormalFrameVector.getFrameVectorCopy();
   }

   public void clear()
   {
      for (YoFramePoint2d contactPoint : contactPoints)
      {
         invalidateContactPoint(contactPoint);
      }
   }
}

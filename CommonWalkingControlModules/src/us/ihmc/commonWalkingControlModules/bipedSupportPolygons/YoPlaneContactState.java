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
   private final List<YoFramePoint2d> contactFramePoints = new ArrayList<YoFramePoint2d>();
   private final List<YoContactPoint> contactPoints = new ArrayList<YoContactPoint>();
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final YoFrameVector contactNormalFrameVector;

   // TODO: Probably get rid of that. Now, it is used for smooth unload/load transitions in the CarIngressEgressController.
   private final DoubleYoVariable wRho;

   @Deprecated
   public YoPlaneContactState(String namePrefix, ReferenceFrame frameAfterJoint, ReferenceFrame planeFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, frameAfterJoint, planeFrame, new ArrayList<FramePoint2d>(), parentRegistry);
   }

   public YoPlaneContactState(String namePrefix, ReferenceFrame frameAfterJoint, ReferenceFrame planeFrame, List<FramePoint2d> contactFramePoints, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.frameAfterJoint = frameAfterJoint;
      this.planeFrame = planeFrame;
      parentRegistry.addChild(registry);
      this.contactNormalFrameVector = new YoFrameVector(namePrefix + "ContactNormalFrameVector", planeFrame, registry);
      
      wRho = new DoubleYoVariable(namePrefix + "_wRhoContactRegularization", registry);
      resetContactRegularization();
      
      for (int i = 0; i < contactFramePoints.size(); i++)
      {
         YoContactPoint contactPoint = new YoContactPoint(namePrefix, i, contactFramePoints.get(i), parentRegistry);
         contactPoints.add(contactPoint);
      }
   }

   @Deprecated
   public void set(List<FramePoint2d> contactFramePoints, double coefficientOfFriction, FrameVector normalContactVector)
   {
      this.contactNormalFrameVector.set(normalContactVector);

      createYoFramePoints(contactFramePoints);

      FramePoint2d temp = new FramePoint2d(planeFrame);
      inContact.set(false);

      for (int i = 0; i < this.contactFramePoints.size(); i++)
      {
         YoFramePoint2d contactPoint = this.contactFramePoints.get(i);
         if (i < contactFramePoints.size())
         {
            FramePoint2d point = contactFramePoints.get(i);
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

   @Deprecated
   public void set(List<FramePoint2d> contactPoints, double coefficientOfFriction)
   {
      set(contactPoints, coefficientOfFriction, new FrameVector(planeFrame, 0.0, 0.0, 1.0));
   }

   private void invalidateContactPoint(YoFramePoint2d contactPoint)
   {
      contactPoint.set(Double.NaN, Double.NaN);
   }

   public List<YoContactPoint> getContactPoints()
   {
      return contactPoints;
   }
   
   public List<FramePoint2d> getContactFramePoints2d()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(contactFramePoints.size());
      for (YoFramePoint2d contactPoint : contactFramePoints)
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
      int oldSize = this.contactFramePoints.size();
      int newSize = contactPoints.size();
      for (int i = oldSize; i < newSize; i++)
      {
         this.contactFramePoints.add(new YoFramePoint2d(namePrefix + "Contact" + i, "", planeFrame, registry));
      }
   }
   
   public List<FramePoint> getContactFramePoints()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactFramePoints.size());
      for (YoFramePoint2d contactPoint : contactFramePoints)
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
      for (YoFramePoint2d contactPoint : contactFramePoints)
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
      for (YoFramePoint2d contactPoint : contactFramePoints)
      {
         invalidateContactPoint(contactPoint);
      }
      inContact.set(false);
   }

   public void setRhoContactRegularization(double wRho)
   {
      this.wRho.set(wRho);
   }

   public double getRhoContactRegularization()
   {
      return wRho.getDoubleValue();
   }

   public void resetContactRegularization()
   {
      wRho.set(DEFAULT_WRHO);
   }
}

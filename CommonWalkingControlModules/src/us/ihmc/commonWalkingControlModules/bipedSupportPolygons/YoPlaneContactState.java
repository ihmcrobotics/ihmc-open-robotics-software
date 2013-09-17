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
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class YoPlaneContactState implements PlaneContactState, ModifiableContactState
{
   private final String namePrefix;
   private final YoVariableRegistry registry;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame planeFrame;
   private final List<FramePoint2d> contactFramePoints = new ArrayList<FramePoint2d>();
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final YoFrameVector contactNormalFrameVector;
   
   private final int totalNumberOfContactPoints;

   private final List<YoContactPoint> contactPoints;
   
   // TODO: Probably get rid of that. Now, it is used for smooth unload/load transitions in the CarIngressEgressController.
   private final DoubleYoVariable wRho;

   @Deprecated
   public YoPlaneContactState(String namePrefix, ReferenceFrame frameAfterJoint, ReferenceFrame planeFrame, double coefficientOfFriction, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, frameAfterJoint, planeFrame, new ArrayList<FramePoint2d>(), coefficientOfFriction, parentRegistry);
   }

   public YoPlaneContactState(String namePrefix, ReferenceFrame bodyFrame, ReferenceFrame planeFrame, List<FramePoint2d> contactFramePoints,
         double coefficientOfFriction, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.coefficientOfFriction.set(coefficientOfFriction);
      this.bodyFrame = bodyFrame;
      this.planeFrame = planeFrame;
      parentRegistry.addChild(registry);
      this.contactNormalFrameVector = new YoFrameVector(namePrefix + "ContactNormalFrameVector", planeFrame, registry);
      
      wRho = new DoubleYoVariable(namePrefix + "_wRhoContactRegularization", registry);
      resetContactRegularization();
      
      contactPoints = new ArrayList<YoContactPoint>(contactFramePoints.size());
      for (int i = 0; i < contactFramePoints.size(); i++)
      {
         YoContactPoint contactPoint = new YoContactPoint(namePrefix, i, contactFramePoints.get(i), parentRegistry);
         contactPoints.add(contactPoint);
      }
      
      totalNumberOfContactPoints = contactPoints.size();
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      this.coefficientOfFriction.set(coefficientOfFriction);
   }
   
   public void setContactNormal(FrameVector normalContactVector)
   {
      this.contactNormalFrameVector.set(normalContactVector);
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
         FramePoint2d contactPoint = this.contactFramePoints.get(i);
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

   @Deprecated
   private void invalidateContactPoint(FramePoint2d contactPoint)
   {
      contactPoint.set(Double.NaN, Double.NaN);
   }

   public List<YoContactPoint> getContactPoints()
   {
      return contactPoints;
   }
   
   @Deprecated
   public List<FramePoint2d> getContactFramePoints2d()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(contactFramePoints.size());
      for (FramePoint2d contactPoint : contactFramePoints)
      {
         if (!contactPoint.containsNaN())
         {
            ret.add(new FramePoint2d(contactPoint));
         }
      }

      return ret;
   }

   public void setContactPointsInContact(boolean[] inContact)
   {
      if (inContact.length != contactPoints.size())
         throw new RuntimeException("Arrays should be of same length!");
      
      for (int i = 0; i < inContact.length; i++)
      {
         setContactPointInContact(i, inContact[i]);
      }
   }

   public void setContactPointInContact(int contactPointIndex, boolean inContact)
   {
      contactPoints.get(contactPointIndex).setInContact(inContact);
   }
   
   public List<FramePoint> getFramePointCopyListInContact()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(getNumberOfPointsInContact());
      
      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);
         
         if (contactPoint.isInContact())
         {
            FramePoint2d framePoint2d = contactPoint.getPosition2d();
            FramePoint framePoint = new FramePoint(framePoint2d.getReferenceFrame(), framePoint2d.getX(), framePoint2d.getY(), 0.0);
            ret.add(framePoint);
         }
      }

      return ret;
   }

   public List<FramePoint2d> getFramePoint2dListInContact()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(getNumberOfPointsInContact());
      
      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoContactPoint contactPoint = contactPoints.get(i);
         
         if (contactPoint.isInContact())
         {
            FramePoint2d framePoint2d = new FramePoint2d(contactPoint.getPosition2d());
            ret.add(framePoint2d);
         }
      }

      return ret;
   }
   
   public int getNumberOfPointsInContact()
   {
      int numberOfPointsInContact = 0;
      
      for (int i = 0; i < contactPoints.size(); i++)
      {
         if (contactPoints.get(i).isInContact())
            numberOfPointsInContact++;
      }
      
      return numberOfPointsInContact;
   }

   public int getTotalNumberOfContactPoints()
   {
      return totalNumberOfContactPoints;
   }

   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   @Deprecated
   private void createYoFramePoints(List<? extends FramePoint2d> contactPoints)
   {
      int oldSize = this.contactFramePoints.size();
      int newSize = contactPoints.size();
      for (int i = oldSize; i < newSize; i++)
      {
         this.contactFramePoints.add(new FramePoint2d(planeFrame));
      }
   }

   @Deprecated
   public List<FramePoint> getContactFramePoints()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactFramePoints.size());
      for (FramePoint2d contactPoint : contactFramePoints)
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

   @Deprecated
   public int getNumberOfContactPoints()
   {
      int ret = 0;
      for (FramePoint2d contactPoint : contactFramePoints)
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
      for (FramePoint2d contactPoint : contactFramePoints)
      {
         invalidateContactPoint(contactPoint);
      }
      
      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).setInContact(false);
      }
      
      inContact.set(false);
   }
   
   public void setFullyConstrained()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).setInContact(true);
         contactFramePoints.set(i, new FramePoint2d(contactPoints.get(i).getPosition2d()));
      }
      
      inContact.set(true);
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

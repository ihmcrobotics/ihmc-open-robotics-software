package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.ArrayList;
import java.util.List;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;

public class YoRollingContactState implements PlaneContactState
{
   private final String namePrefix;
   private final YoVariableRegistry registry;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame updatableContactFrame;
   private final List<YoFramePoint2d> contactPoints = new ArrayList<YoFramePoint2d>();
   private final BooleanYoVariable inContact;
   private final DoubleYoVariable coefficientOfFriction;
   private final Transform3D transformFromContactFrameToBodyFrame = new Transform3D();
   private final ArrayList<DynamicGraphicPosition> contactPointGraphics = new ArrayList<DynamicGraphicPosition>();
   private final ContactableCylinderBody contactableCylinderBody;
   private final FrameVector contactNormalFrameVector;

   // Class enabling to update the contact points of a contactable cylindrical body as it is rolling on the ground or on another contactable surface 
   
   public YoRollingContactState(String namePrefix, ContactableCylinderBody contactableCylinderBody, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, contactableCylinderBody, parentRegistry, null);
   }
   
   public YoRollingContactState(String namePrefix, ContactableCylinderBody contactableCylinderBody, YoVariableRegistry parentRegistry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.namePrefix = namePrefix;
      // The rolling contactable body
      this.contactableCylinderBody = contactableCylinderBody;
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.inContact = new BooleanYoVariable(namePrefix + "InContact", registry);
      this.coefficientOfFriction = new DoubleYoVariable(namePrefix + "CoefficientOfFriction", registry);
      this.updatableContactFrame = new ReferenceFrame(namePrefix + "ContactFrame", getBodyFrame())
      {
         private static final long serialVersionUID = 6993243554111815201L;

         @Override
         public void updateTransformToParent(Transform3D transformToParent)
         {
            transformToParent.set(transformFromContactFrameToBodyFrame);
         }
      };
      
      setContactPoints(contactableCylinderBody.getContactPoints2d());
      
      if (dynamicGraphicObjectsListRegistry != null)
      {
         for (int i = 0; i < contactPoints.size(); i++)
         {
            contactPointGraphics.add(new DynamicGraphicPosition(namePrefix + "GC", String.valueOf(i), registry, 0.01, YoAppearance.Red()));
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("thighContactPoints", contactPointGraphics.get(i));
            dynamicGraphicObjectsListRegistry.registerArtifact("thighContactPoints", contactPointGraphics.get(i).createArtifact());
         }
      }

      updateContactPoints();
      
      parentRegistry.addChild(registry);
      
      this.contactNormalFrameVector = new FrameVector(updatableContactFrame, 0.0, 0.0, 1.0);
   }

   private void setContactPoints(List<FramePoint2d> contactPoints)
   {
      createYoFramePoints(contactPoints);

      FramePoint2d temp = new FramePoint2d(updatableContactFrame);
      inContact.set(false);

      for (int i = 0; i < this.contactPoints.size(); i++)
      {
         if (i < contactPoints.size())
         {
            FramePoint2d point = contactPoints.get(i);
            temp.setAndChangeFrame(point);
            temp.changeFrame(updatableContactFrame);
            this.contactPoints.get(i).set(temp);
            inContact.set(true);
         }
         else
         {
            this.contactPoints.get(i).set(Double.NaN, Double.NaN);
         }
      }
   }

   public void setCoefficientOfFriction(double coefficientOfFriction)
   {
      if (coefficientOfFriction < 0.0)
         throw new RuntimeException("Coefficient of friction is negative: " + coefficientOfFriction);
      
      this.coefficientOfFriction.set(coefficientOfFriction);
   }

   public void updateContactPoints()
   {
      // The contact reference frame is updated such as:
      // 1- it remains tangential to the contactable cylindrical body,
      // 2- it remains under the contactable cylindrical body (at the lowest height)
      Transform3D transformFromRigiBodyToWorld = getBodyFrame().getTransformToDesiredFrame(worldFrame );
      Matrix3d rotationFromRigiBodyToWorld = new Matrix3d();
      transformFromRigiBodyToWorld.get(rotationFromRigiBodyToWorld);

      // Look for the angle theta that will position the reference contact frame. The contact points will be automatically positioned as they are expressed in that reference frame.
      // Ex. for the thigh: theta == 0 => back of the thigh, theta == PI/2 => left side of the thigh (whatever it is the left or right thigh)
      double theta = -Math.PI / 2.0 + Math.atan2(rotationFromRigiBodyToWorld.m20, rotationFromRigiBodyToWorld.m21);

      transformFromContactFrameToBodyFrame.setIdentity();
      Vector3d eulerAngles = new Vector3d(theta, Math.PI / 2.0, 0.0);
      transformFromContactFrameToBodyFrame.setEuler(eulerAngles);
      FramePoint originInBodyFrame = contactableCylinderBody.getCopyOfCylinderOriginInBodyFrame();
      double cylinderRadius = contactableCylinderBody.getCylinderRadius();
      Vector3d translation = new Vector3d(-cylinderRadius  * Math.cos(theta) + originInBodyFrame .getX(), cylinderRadius * Math.sin(theta)
            + originInBodyFrame.getY(), originInBodyFrame.getZ());
      transformFromContactFrameToBodyFrame.setTranslation(translation);

      updatableContactFrame.update();
      
      // Updating the dynamic graphic object representing the contact points location
      for (int i = 0; i < contactPointGraphics.size(); i++)
      {
         FramePoint point = new FramePoint(getContactPoints().get(i));
         point.changeFrame(worldFrame);
         contactPointGraphics.get(i).setPosition(point);
      }
   }
   
   public List<FramePoint2d> getContactPoints2d()
   {
      List<FramePoint2d> ret = new ArrayList<FramePoint2d>(contactPoints.size());
      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoFramePoint2d contactPoint = contactPoints.get(i);
         if (!contactPoint .containsNaN())
         {
            ret.add(contactPoint.getFramePoint2dCopy());
         }
      }

      return ret;
   }

   public ReferenceFrame getBodyFrame()
   {
      return contactableCylinderBody.getBodyFrame();
   }

   private void createYoFramePoints(List<? extends FramePoint2d> contactPoints)
   {
      int oldSize = this.contactPoints.size();
      int newSize = contactPoints.size();
      for (int i = oldSize; i < newSize; i++)
      {
         this.contactPoints.add(new YoFramePoint2d(namePrefix + "Contact" + i, "", updatableContactFrame, registry));
      }
   }

   public List<FramePoint> getContactPoints()
   {
      List<FramePoint> ret = new ArrayList<FramePoint>(contactPoints.size());
      for (int i = 0; i < contactPoints.size(); i++)
      {
         YoFramePoint2d contactPoint = contactPoints.get(i);
         if (!contactPoint.containsNaN())
         {
            ret.add(new FramePoint(contactPoint.getReferenceFrame(), contactPoint.getX(), contactPoint.getY(), 0.0));
         }
      }

      return ret;
   }

   public ReferenceFrame getPlaneFrame()
   {
      return updatableContactFrame;
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
      return contactNormalFrameVector;
   }
}

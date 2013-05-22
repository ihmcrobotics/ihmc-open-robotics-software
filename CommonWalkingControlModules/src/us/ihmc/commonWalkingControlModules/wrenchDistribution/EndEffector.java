package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class EndEffector
{
   // TODO: YoVariableize the boolean?
   private OptimizerContactModel contactModel;
   private final ReferenceFrame referenceFrame;
   private boolean loadBearing;
   private final EndEffectorOutput output;

   public EndEffector(ReferenceFrame centerOfMassFrame, ReferenceFrame endEffectorFrame, YoVariableRegistry registry)
   {
      this("", centerOfMassFrame, endEffectorFrame, registry);
   }

   public EndEffector(String nameSuffix, ReferenceFrame centerOfMassFrame, ReferenceFrame endEffectorFrame, YoVariableRegistry registry)
   {
      this.referenceFrame = endEffectorFrame;
      this.output = new EndEffectorOutput(nameSuffix, centerOfMassFrame, endEffectorFrame, registry);
   }

   public OptimizerContactModel getContactModel()
   {
      return contactModel;
   }

   public void setContactModel(OptimizerContactModel contactModel)
   {
      this.contactModel = contactModel;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public boolean isLoadBearing()
   {
      return loadBearing;
   }

   public void setLoadBearing(boolean inContact)
   {
      this.loadBearing = inContact;
   }

   public EndEffectorOutput getOutput()
   {
      return this.output;
   }

   public static EndEffector fromCylinder(ReferenceFrame centerOfMassFrame, CylindricalContactState cylinder, YoVariableRegistry registry)
   {
      EndEffector ret = new EndEffector(centerOfMassFrame, cylinder.getEndEffectorFrame(), registry);
      OptimizerCylinderContactModel cylinderContactModel = new OptimizerCylinderContactModel();
      cylinderContactModel.setup(cylinder.getCoefficientOfFriction(), cylinder.getCylinderRadius(), cylinder.getHalfHandWidth(),
                                 cylinder.getTensileGripForce(), cylinder.getGripWeaknessFactor(), cylinder.getCylinderFrame());
      ret.setContactModel(cylinderContactModel);
      ret.setLoadBearing(cylinder.isInContact());

      return ret;
   }

   public static EndEffector fromPlane(String nameSuffix, ReferenceFrame centerOfMassFrame, PlaneContactState plane, YoVariableRegistry registry)
   {
      EndEffector ret = new EndEffector(nameSuffix, centerOfMassFrame, plane.getBodyFrame(), registry);
      OptimizerPlaneContactModel model = new OptimizerPlaneContactModel();
      model.setup(plane.getCoefficientOfFriction(), plane.getContactPoints(), plane.getBodyFrame());
      ret.setContactModel(model);
      ret.setLoadBearing(plane.inContact());

      return ret;
   }

   public void updateFromCylinder(CylindricalContactState cylinder)
   {
      referenceFrame.checkReferenceFrameMatch(cylinder.getEndEffectorFrame());
      OptimizerCylinderContactModel optimizerModel = (OptimizerCylinderContactModel) this.getContactModel();
      optimizerModel.setup(cylinder.getCoefficientOfFriction(), cylinder.getCylinderRadius(), cylinder.getHalfHandWidth(), cylinder.getTensileGripForce(),
                           cylinder.getGripWeaknessFactor(), cylinder.getCylinderFrame());
      this.setLoadBearing(cylinder.isInContact());
   }

   public void updateFromPlane(PlaneContactState plane)
   {
      referenceFrame.checkReferenceFrameMatch(plane.getBodyFrame());
      OptimizerPlaneContactModel optimizerPlaneContactModel = (OptimizerPlaneContactModel) this.getContactModel();
      optimizerPlaneContactModel.setup(plane.getCoefficientOfFriction(), plane.getContactPoints(), plane.getBodyFrame());
      this.setLoadBearing(plane.inContact());
   }

}

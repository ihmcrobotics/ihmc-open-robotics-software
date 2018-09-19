package us.ihmc.simulationConstructionSetTools.util.groundContact;

import java.util.ArrayList;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ZHeightContactPointDisablerGroundContactModel implements GroundContactModel
{
   private static final long serialVersionUID = -6870738695586772779L;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("contactPointDisabler");
   private final YoDouble zHeightDisableThreshold = new YoDouble("zHeightDisableThreshold", registry);
   private final GroundContactModel baseModel;

   private ArrayList<GroundContactPoint> contactPoints;
   
   public ZHeightContactPointDisablerGroundContactModel(GroundContactModel baseModel, ArrayList<GroundContactPoint> contactPoints, double zHeightDisableThreshold, YoVariableRegistry parentRegistry)
   {
      this.baseModel = baseModel;
      this.contactPoints = contactPoints;
      this.zHeightDisableThreshold.set(zHeightDisableThreshold);
      parentRegistry.addChild(registry);
   }
   
   @Override
   public void doGroundContact()
   {
      updateContactPoints();
      baseModel.doGroundContact();
   }
   
   private void updateContactPoints()
   {
      for(int i = 0; i < contactPoints.size(); i++)
      {
         GroundContactPoint groundContactPoint = contactPoints.get(i);
         boolean isAboveDisableThreshold = groundContactPoint.getZ() > zHeightDisableThreshold.getDoubleValue();
         
         if(isAboveDisableThreshold && !groundContactPoint.isDisabled())
         {
            groundContactPoint.disable();
         }
         else if(!isAboveDisableThreshold && groundContactPoint.isDisabled())
         {
            groundContactPoint.setNotInContact();
         }
      }
   }
   
   @Override
   public GroundProfile3D getGroundProfile3D()
   {
      return baseModel.getGroundProfile3D();
   }
   
   @Override
   public void setGroundProfile3D(GroundProfile3D groundProfile3D)
   {
      baseModel.setGroundProfile3D(groundProfile3D);
   }
}

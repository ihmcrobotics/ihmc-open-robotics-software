package us.ihmc.commonWalkingControlModules.momentumBasedController;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepUtils;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * @author twan
 *         Date: 5/11/13
 */
public class PlaneContactWrenchProcessor
{
   private final List<ContactablePlaneBody> contactablePlaneBodies;
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   private final LinkedHashMap<ContactablePlaneBody, DoubleYoVariable> normalTorques = new LinkedHashMap<ContactablePlaneBody, DoubleYoVariable>();
   private final LinkedHashMap<ContactablePlaneBody, DoubleYoVariable> groundReactionForceMagnitudes = new LinkedHashMap<ContactablePlaneBody, DoubleYoVariable>();
   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint> centersOfPressureWorld = new LinkedHashMap<ContactablePlaneBody, YoFramePoint>();
   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint2d> centersOfPressure2d = new LinkedHashMap<ContactablePlaneBody, YoFramePoint2d>();

   private final Map<ContactablePlaneBody, FramePoint2d> cops = new LinkedHashMap<ContactablePlaneBody, FramePoint2d>();

   public PlaneContactWrenchProcessor(List<ContactablePlaneBody> contactablePlaneBodies, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry registry)
   {
      this.contactablePlaneBodies = contactablePlaneBodies;
      for (ContactablePlaneBody contactableBody : contactablePlaneBodies)
      {
         DoubleYoVariable forceMagnitude = new DoubleYoVariable(contactableBody.getRigidBody().getName() + "ForceMagnitude", registry);
         groundReactionForceMagnitudes.put(contactableBody, forceMagnitude);

         DoubleYoVariable normalTorque = new DoubleYoVariable(contactableBody.getRigidBody().getName() + "NormalTorque", registry);
         normalTorques.put(contactableBody, normalTorque);

         String copName = contactableBody.getRigidBody().getName() + "CoP";
         String listName = "cops";

         YoFramePoint2d cop2d = new YoFramePoint2d(copName + "2d", "", contactableBody.getPlaneFrame(), registry);
         centersOfPressure2d.put(contactableBody, cop2d);

         YoFramePoint cop = new YoFramePoint(copName, ReferenceFrame.getWorldFrame(), registry);
         centersOfPressureWorld.put(contactableBody, cop);

         if (dynamicGraphicObjectsListRegistry != null)
         {
            DynamicGraphicPosition copViz = cop.createDynamicGraphicPosition(copName, 0.005, YoAppearance.Navy(), DynamicGraphicPosition.GraphicType.BALL);
            dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, copViz);
            dynamicGraphicObjectsListRegistry.registerArtifact(listName, copViz.createArtifact());
         }
      }
   }

   public void compute(Map<RigidBody, Wrench> externalWrenches)
   {
      cops.clear();
      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         Wrench wrench = externalWrenches.get(contactablePlaneBody.getRigidBody());

         if (wrench != null)
         {
            FrameVector force = wrench.getLinearPartAsFrameVectorCopy();

            FramePoint2d cop = new FramePoint2d(ReferenceFrame.getWorldFrame());
            double normalTorque = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, contactablePlaneBody.getPlaneFrame());
            cops.put(contactablePlaneBody, cop);

            centersOfPressure2d.get(contactablePlaneBody).set(cop);

            FramePoint cop3d = cop.toFramePoint();
            cop3d.changeFrame(ReferenceFrame.getWorldFrame());

            centersOfPressureWorld.get(contactablePlaneBody).set(cop3d);
            groundReactionForceMagnitudes.get(contactablePlaneBody).set(force.length());
            normalTorques.get(contactablePlaneBody).set(normalTorque);
         }
         else
         {
            groundReactionForceMagnitudes.get(contactablePlaneBody).set(0.0);

//          centersOfPressure2d.get(contactablePlaneBody).set(Double.NaN, Double.NaN);
            centersOfPressureWorld.get(contactablePlaneBody).setToNaN();
         }
      }
   }

   public Map<ContactablePlaneBody, FramePoint2d> getCops()
   {
      return cops;
   }

   public void initialize()
   {
      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         FramePoint footCenter = FootstepUtils.calculateFootCenter(contactablePlaneBody);
         footCenter.changeFrame(contactablePlaneBody.getPlaneFrame());
         cops.put(contactablePlaneBody, footCenter.toFramePoint2d());
      }

   }
}

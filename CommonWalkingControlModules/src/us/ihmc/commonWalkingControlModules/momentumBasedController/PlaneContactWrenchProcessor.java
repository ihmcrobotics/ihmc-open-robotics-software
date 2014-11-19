package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

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
   private final YoVariableRegistry registry;

   public PlaneContactWrenchProcessor(List<ContactablePlaneBody> contactablePlaneBodies, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(getClass().getSimpleName());
      this.contactablePlaneBodies = contactablePlaneBodies;
      for (ContactablePlaneBody contactableBody : contactablePlaneBodies)
      {
         String name = contactableBody.getSoleFrame().getName();
         DoubleYoVariable forceMagnitude = new DoubleYoVariable(name + "ForceMagnitude", registry);
         groundReactionForceMagnitudes.put(contactableBody, forceMagnitude);

         DoubleYoVariable normalTorque = new DoubleYoVariable(name + "NormalTorque", registry);
         normalTorques.put(contactableBody, normalTorque);

         String copName = name + "CoP";
         String listName = "cops";

         YoFramePoint2d cop2d = new YoFramePoint2d(copName + "2d", "", contactableBody.getSoleFrame(), registry);
         centersOfPressure2d.put(contactableBody, cop2d);

         YoFramePoint cop = new YoFramePoint(copName, ReferenceFrame.getWorldFrame(), registry);
         centersOfPressureWorld.put(contactableBody, cop);

         if (yoGraphicsListRegistry != null)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(copName, cop, 0.005, YoAppearance.Navy(), YoGraphicPosition.GraphicType.BALL);
            yoGraphicsListRegistry.registerYoGraphic(listName, copViz);
            yoGraphicsListRegistry.registerArtifact(listName, copViz.createArtifact());
         }
      }

      parentRegistry.addChild(registry);
   }

   public void compute(Map<RigidBody, Wrench> externalWrenches)
   {
      cops.clear();
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         Wrench wrench = externalWrenches.get(contactablePlaneBody.getRigidBody());

         if (wrench != null)
         {
            FrameVector force = wrench.getLinearPartAsFrameVectorCopy();

            FramePoint2d cop = new FramePoint2d(ReferenceFrame.getWorldFrame());
            double normalTorque = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, contactablePlaneBody.getSoleFrame());
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
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         FramePoint2d footCenter2d = new FramePoint2d(contactablePlaneBody.getSoleFrame());
         cops.put(contactablePlaneBody, footCenter2d);
      }
   }
}

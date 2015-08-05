package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
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

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final LinkedHashMap<ContactablePlaneBody, DoubleYoVariable> normalTorques = new LinkedHashMap<>();
   private final LinkedHashMap<ContactablePlaneBody, DoubleYoVariable> groundReactionForceMagnitudes = new LinkedHashMap<>();
   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint> centersOfPressureWorld = new LinkedHashMap<>();
   private final LinkedHashMap<ContactablePlaneBody, YoFramePoint2d> centersOfPressure2d = new LinkedHashMap<>();

   private final Map<ContactablePlaneBody, FramePoint2d> cops = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoFramePoint2d> yoCops = new LinkedHashMap<>();

   public PlaneContactWrenchProcessor(List<ContactablePlaneBody> contactablePlaneBodies, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
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

         FramePoint2d footCenter2d = new FramePoint2d(contactableBody.getSoleFrame());
         footCenter2d.setToNaN();
         cops.put(contactableBody, footCenter2d);
      
         
         YoFramePoint2d yoCop = new YoFramePoint2d(contactableBody.getName() + "CoP", contactableBody.getSoleFrame(), registry);
         yoCop.set(footCenter2d);
         yoCops.put(contactableBody, yoCop);
         
         if (yoGraphicsListRegistry != null)
         {
            YoGraphicPosition copViz = new YoGraphicPosition(copName, cop, 0.005, YoAppearance.Navy(), YoGraphicPosition.GraphicType.BALL);
            yoGraphicsListRegistry.registerYoGraphic(listName, copViz);
            yoGraphicsListRegistry.registerArtifact(listName, copViz.createArtifact());
         }
      }

      parentRegistry.addChild(registry);
   }

   private final FramePoint tempCoP3d = new FramePoint();
   private final FrameVector tempForce = new FrameVector();
   
   public void compute(Map<RigidBody, Wrench> externalWrenches)
   {
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         FramePoint2d cop = cops.get(contactablePlaneBody);
         YoFramePoint2d yoCop = yoCops.get(contactablePlaneBody);
         yoCop.getFrameTuple2d(cop);
         
         Wrench wrench = externalWrenches.get(contactablePlaneBody.getRigidBody());

         if (wrench != null)
         {
            wrench.packLinearPartIncludingFrame(tempForce);

            double normalTorque = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, contactablePlaneBody.getSoleFrame());

            centersOfPressure2d.get(contactablePlaneBody).set(cop);

            tempCoP3d.setXYIncludingFrame(cop);
            centersOfPressureWorld.get(contactablePlaneBody).setAndMatchFrame(tempCoP3d);
            groundReactionForceMagnitudes.get(contactablePlaneBody).set(tempForce.length());
            normalTorques.get(contactablePlaneBody).set(normalTorque);
         }
         else
         {
            groundReactionForceMagnitudes.get(contactablePlaneBody).set(0.0);
            centersOfPressureWorld.get(contactablePlaneBody).setToNaN();
            cop.setToNaN();
         }
         
         yoCop.set(cop);
      }
   }

   public void initialize()
   {
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         cops.get(contactablePlaneBody).setToZero((contactablePlaneBody.getSoleFrame()));
      }
   }

   public FramePoint2d getCoP(ContactablePlaneBody contactablePlaneBody)
   {
      YoFramePoint2d yoCop = yoCops.get(contactablePlaneBody);
      FramePoint2d cop = cops.get(contactablePlaneBody);
      
      yoCop.getFrameTuple2d(cop);
      return cop;
   }
}

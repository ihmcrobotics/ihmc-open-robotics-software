package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 5/11/13
 */
public class PlaneContactWrenchProcessor
{
   private final static boolean VISUALIZE = false;

   private final List<? extends ContactablePlaneBody> contactablePlaneBodies;
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final Map<ContactablePlaneBody, DoubleYoVariable> normalTorques = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, DoubleYoVariable> groundReactionForceMagnitudes = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoFramePoint> centersOfPressureWorld = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoFramePoint2d> centersOfPressure2d = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoFramePoint2d> yoCops = new LinkedHashMap<>();

   private final Map<ContactablePlaneBody, FramePoint2d> cops = new LinkedHashMap<>();

   private final CenterOfPressureDataHolder desiredCenterOfPressureDataHolder;

   public PlaneContactWrenchProcessor(List<? extends ContactablePlaneBody> contactablePlaneBodies, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      Map<RigidBody, ReferenceFrame> soleFrames = new HashMap<>();

      this.contactablePlaneBodies = contactablePlaneBodies;
      for (ContactablePlaneBody contactableBody : contactablePlaneBodies)
      {
         soleFrames.put(contactableBody.getRigidBody(), contactableBody.getSoleFrame());

         String name = contactableBody.getSoleFrame().getName();
         DoubleYoVariable forceMagnitude = new DoubleYoVariable(name + "ForceMagnitude", registry);
         groundReactionForceMagnitudes.put(contactableBody, forceMagnitude);

         DoubleYoVariable normalTorque = new DoubleYoVariable(name + "NormalTorque", registry);
         normalTorques.put(contactableBody, normalTorque);

         String copName = name + "CoP";
         String listName = getClass().getSimpleName();

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
            copViz.setVisible(VISUALIZE);
            yoGraphicsListRegistry.registerYoGraphic(listName, copViz);
            YoArtifactPosition artifact = copViz.createArtifact();
            artifact.setVisible(VISUALIZE);
            yoGraphicsListRegistry.registerArtifact(listName, artifact);
         }
      }

      desiredCenterOfPressureDataHolder = new CenterOfPressureDataHolder(soleFrames);

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
            wrench.getLinearPartIncludingFrame(tempForce);

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
         desiredCenterOfPressureDataHolder.setCenterOfPressure(cop, contactablePlaneBody.getRigidBody());
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

   public void getDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2d desiredCoPToPack)
   {
      YoFramePoint2d yoCop = yoCops.get(contactablePlaneBody);
      yoCop.getFrameTuple2dIncludingFrame(desiredCoPToPack);
   }

   public CenterOfPressureDataHolder getDesiredCenterOfPressureDataHolder()
   {
      return desiredCenterOfPressureDataHolder;
   }
}

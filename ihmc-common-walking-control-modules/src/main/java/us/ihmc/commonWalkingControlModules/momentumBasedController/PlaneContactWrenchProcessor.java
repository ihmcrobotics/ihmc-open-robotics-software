package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

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
   private final Map<ContactablePlaneBody, YoDouble> normalTorques = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoDouble> groundReactionForceMagnitudes = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoFramePoint3D> centersOfPressureWorld = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoFramePoint2D> centersOfPressure2d = new LinkedHashMap<>();
   private final Map<ContactablePlaneBody, YoFramePoint2D> yoCops = new LinkedHashMap<>();

   private final Map<ContactablePlaneBody, FramePoint2D> cops = new LinkedHashMap<>();

   private final CenterOfPressureDataHolder desiredCenterOfPressureDataHolder;

   public PlaneContactWrenchProcessor(List<? extends ContactablePlaneBody> contactablePlaneBodies, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      List<RigidBodyBasics> feet = new ArrayList<>();

      this.contactablePlaneBodies = contactablePlaneBodies;
      for (ContactablePlaneBody contactableBody : contactablePlaneBodies)
      {
         feet.add(contactableBody.getRigidBody());

         String name = contactableBody.getSoleFrame().getName();
         YoDouble forceMagnitude = new YoDouble(name + "ForceMagnitude", registry);
         groundReactionForceMagnitudes.put(contactableBody, forceMagnitude);

         YoDouble normalTorque = new YoDouble(name + "NormalTorque", registry);
         normalTorques.put(contactableBody, normalTorque);

         String copName = name + "CoP";
         String listName = getClass().getSimpleName();

         YoFramePoint2D cop2d = new YoFramePoint2D(copName + "2d", "", contactableBody.getSoleFrame(), registry);
         centersOfPressure2d.put(contactableBody, cop2d);

         YoFramePoint3D cop = new YoFramePoint3D(copName, ReferenceFrame.getWorldFrame(), registry);
         centersOfPressureWorld.put(contactableBody, cop);

         FramePoint2D footCenter2d = new FramePoint2D(contactableBody.getSoleFrame());
         footCenter2d.setToNaN();
         cops.put(contactableBody, footCenter2d);

         YoFramePoint2D yoCop = new YoFramePoint2D(contactableBody.getName() + "CoP", contactableBody.getSoleFrame(), registry);
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

      desiredCenterOfPressureDataHolder = new CenterOfPressureDataHolder(feet);

      parentRegistry.addChild(registry);
   }

   private final FramePoint3D tempCoP3d = new FramePoint3D();
   private final FrameVector3D tempForce = new FrameVector3D();

   public void compute(Map<RigidBodyBasics, Wrench> externalWrenches)
   {
      for (int i = 0; i < contactablePlaneBodies.size(); i++)
      {
         ContactablePlaneBody contactablePlaneBody = contactablePlaneBodies.get(i);
         FramePoint2D cop = cops.get(contactablePlaneBody);
         YoFramePoint2D yoCop = yoCops.get(contactablePlaneBody);
         cop.set(yoCop);

         Wrench wrench = externalWrenches.get(contactablePlaneBody.getRigidBody());

         if (wrench != null)
         {
            tempForce.setIncludingFrame(wrench.getLinearPart());

            double normalTorque = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(cop, wrench, contactablePlaneBody.getSoleFrame());

            centersOfPressure2d.get(contactablePlaneBody).set(cop);

            tempCoP3d.setIncludingFrame(cop, 0.0);
            centersOfPressureWorld.get(contactablePlaneBody).setMatchingFrame(tempCoP3d);
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

   public void getDesiredCenterOfPressure(ContactablePlaneBody contactablePlaneBody, FramePoint2D desiredCoPToPack)
   {
      YoFramePoint2D yoCop = yoCops.get(contactablePlaneBody);
      desiredCoPToPack.setIncludingFrame(yoCop);
   }

   public CenterOfPressureDataHolder getDesiredCenterOfPressureDataHolder()
   {
      return desiredCenterOfPressureDataHolder;
   }
}

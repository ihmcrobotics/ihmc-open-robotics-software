package us.ihmc.commonWalkingControlModules.visualizer;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicArrow3D;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicListDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * @author twan Date: 6/2/13
 */
public class WrenchVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double FORCE_VECTOR_SCALE = 0.0015;
   private static final double TORQUE_VECTOR_SCALE = 0.0015;

   private final String name;
   private final double vizScaling;
   private final AppearanceDefinition forceAppearance;
   private final AppearanceDefinition torqueAppearance;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final List<RigidBodyBasics> rigidBodies = new ArrayList<>();
   private final Map<RigidBodyBasics, SingleWrenchVisualizer> visualizerMap = new LinkedHashMap<>();

   public WrenchVisualizer(String name, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry parentRegistry)
   {
      this(name, vizScaling, yoGraphicsListRegistry, parentRegistry, YoAppearance.OrangeRed(), YoAppearance.CornflowerBlue());
   }

   public WrenchVisualizer(String name,
                           double vizScaling,
                           YoGraphicsListRegistry yoGraphicsListRegistry,
                           YoRegistry parentRegistry,
                           AppearanceDefinition forceAppearance,
                           AppearanceDefinition torqueAppearance)
   {
      this.name = name;
      this.vizScaling = vizScaling;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.forceAppearance = forceAppearance;
      this.torqueAppearance = torqueAppearance;

      parentRegistry.addChild(registry);
   }

   public void registerRigidBodies(Collection<? extends RigidBodyBasics> rigidBodies)
   {
      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         registerRigidBody(rigidBody);
      }
   }

   public void registerRigidBody(RigidBodyBasics rigidBody)
   {
      rigidBodies.add(rigidBody);
      visualizerMap.put(rigidBody, new RigidBodyWrenchVisualizer(rigidBody));
   }

   public void registerContactableBodies(Collection<? extends ContactableBody> contactableBodies)
   {
      for (ContactableBody contactableBody : contactableBodies)
      {
         registerContactableBody(contactableBody);
      }
   }

   public void registerContactableBody(ContactableBody contactableBody)
   {
      registerRigidBody(contactableBody.getRigidBody());
   }

   public void registerContactablePlaneBodies(Collection<? extends ContactablePlaneBody> contactablePlaneBodies)
   {
      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         registerContactablePlaneBody(contactablePlaneBody);
      }
   }

   public void registerContactablePlaneBody(ContactablePlaneBody contactablePlaneBody)
   {
      RigidBodyBasics rigidBody = contactablePlaneBody.getRigidBody();
      rigidBodies.add(rigidBody);
      visualizerMap.put(rigidBody, new ContactablePlaneBodyWrenchVisualizer(contactablePlaneBody));
   }

   public void visualize(Map<RigidBodyBasics, ? extends WrenchReadOnly> wrenches)
   {
      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         WrenchReadOnly wrench = wrenches.get(rigidBody);
         visualizerMap.get(rigidBody).accept(wrench);
      }
   }

   private interface SingleWrenchVisualizer extends Consumer<WrenchReadOnly>, SCS2YoGraphicHolder
   {
   }

   private final Wrench tempWrench = new Wrench();
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   private class ContactablePlaneBodyWrenchVisualizer implements SingleWrenchVisualizer
   {
      private final YoFrameVector3D force;
      private final YoFrameVector3D torque;
      private final YoFramePoint3D centerOfPressure;
      private final ReferenceFrame soleFrame;

      public ContactablePlaneBodyWrenchVisualizer(ContactablePlaneBody contactablePlaneBody)
      {
         String prefix = name + contactablePlaneBody.getName();
         force = new YoFrameVector3D(prefix + "Force", worldFrame, registry);
         torque = new YoFrameVector3D(prefix + "Torque", worldFrame, registry);
         centerOfPressure = new YoFramePoint3D(prefix + "CenterOfPressure", worldFrame, registry);
         soleFrame = contactablePlaneBody.getSoleFrame();

         double forceScale = FORCE_VECTOR_SCALE * vizScaling;
         double torqueScale = TORQUE_VECTOR_SCALE * vizScaling;
         YoGraphicVector forceViz = new YoGraphicVector(prefix + "ForceViz", centerOfPressure, force, forceScale, forceAppearance, true);
         YoGraphicVector torqueViz = new YoGraphicVector(prefix + "TorqueViz", centerOfPressure, torque, torqueScale, torqueAppearance, true);

         yoGraphicsListRegistry.registerYoGraphic(name, forceViz);
         yoGraphicsListRegistry.registerYoGraphic(name, torqueViz);

      }

      @Override
      public void accept(WrenchReadOnly wrench)
      {
         if (wrench != null)
         {
            tempWrench.setIncludingFrame(wrench);
            tempWrench.changeFrame(soleFrame);
            double torqueZ = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure, tempWrench, soleFrame);

            force.setMatchingFrame(tempWrench.getLinearPart());
            torque.set(0.0, 0.0, torqueZ);
            soleFrame.transformFromThisToDesiredFrame(worldFrame, torque);
         }
         else
         {
            force.set(Double.NaN, Double.NaN, Double.NaN);
            torque.set(Double.NaN, Double.NaN, Double.NaN);
            centerOfPressure.set(Double.NaN, Double.NaN, Double.NaN);
         }
      }

      public YoGraphicDefinition getSCS2YoGraphics()
      {
         return new YoGraphicListDefinition(newYoGraphicArrow3D(force.getNamePrefix(),
                                                                centerOfPressure,
                                                                force,
                                                                FORCE_VECTOR_SCALE * vizScaling,
                                                                ColorDefinitions.OrangeRed()),
                                            newYoGraphicArrow3D(torque.getNamePrefix(),
                                                                centerOfPressure,
                                                                torque,
                                                                TORQUE_VECTOR_SCALE * vizScaling,
                                                                ColorDefinitions.CornflowerBlue()));
      }
   }

   private class RigidBodyWrenchVisualizer implements SingleWrenchVisualizer
   {
      private final YoFrameVector3D force;
      private final YoFrameVector3D torque;
      private final YoFramePoint3D pointOfApplication;

      public RigidBodyWrenchVisualizer(RigidBodyReadOnly rigidBody)
      {
         String prefix = name + rigidBody.getName();
         force = new YoFrameVector3D(prefix + "Force", worldFrame, registry);
         torque = new YoFrameVector3D(prefix + "Torque", worldFrame, registry);
         pointOfApplication = new YoFramePoint3D(prefix + "PointOfApplication", worldFrame, registry);

         double forceScale = FORCE_VECTOR_SCALE * vizScaling;
         double torqueScale = TORQUE_VECTOR_SCALE * vizScaling;
         YoGraphicVector forceViz = new YoGraphicVector(prefix + "ForceViz", pointOfApplication, force, forceScale, forceAppearance, true);
         YoGraphicVector torqueViz = new YoGraphicVector(prefix + "TorqueViz", pointOfApplication, torque, torqueScale, torqueAppearance, true);

         yoGraphicsListRegistry.registerYoGraphic(name, forceViz);
         yoGraphicsListRegistry.registerYoGraphic(name, torqueViz);
      }

      @Override
      public void accept(WrenchReadOnly wrench)
      {
         if (wrench != null)
         {
            tempWrench.setIncludingFrame(wrench);
            tempWrench.changeFrame(tempWrench.getBodyFrame());

            force.setMatchingFrame(tempWrench.getLinearPart());
            torque.setMatchingFrame(tempWrench.getAngularPart());
            pointOfApplication.setFromReferenceFrame(wrench.getBodyFrame());
         }
         else
         {
            force.set(Double.NaN, Double.NaN, Double.NaN);
            torque.set(Double.NaN, Double.NaN, Double.NaN);
            pointOfApplication.set(Double.NaN, Double.NaN, Double.NaN);
         }
      }

      public YoGraphicDefinition getSCS2YoGraphics()
      {
         return new YoGraphicListDefinition(newYoGraphicArrow3D(force.getNamePrefix(),
                                                                pointOfApplication,
                                                                force,
                                                                FORCE_VECTOR_SCALE * vizScaling,
                                                                ColorDefinitions.OrangeRed()),
                                            newYoGraphicArrow3D(torque.getNamePrefix(),
                                                                pointOfApplication,
                                                                torque,
                                                                TORQUE_VECTOR_SCALE * vizScaling,
                                                                ColorDefinitions.CornflowerBlue()));
      }
   }

   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (SingleWrenchVisualizer visualizer : visualizerMap.values())
      {
         group.addChild(visualizer.getSCS2YoGraphics());
      }
      return group;
   }
}

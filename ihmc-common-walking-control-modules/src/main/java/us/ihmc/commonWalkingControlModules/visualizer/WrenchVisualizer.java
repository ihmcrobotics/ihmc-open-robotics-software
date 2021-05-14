package us.ihmc.commonWalkingControlModules.visualizer;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.contactable.ContactableBody;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * @author twan
 *         Date: 6/2/13
 */
public class WrenchVisualizer
{
   private static final double FORCE_VECTOR_SCALE = 0.0015;
   private static final double TORQUE_VECTOR_SCALE = 0.0015;

   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final Map<RigidBodyBasics, YoFrameVector3D> forces = new LinkedHashMap<RigidBodyBasics, YoFrameVector3D>();
   private final Map<RigidBodyBasics, YoFrameVector3D> torques = new LinkedHashMap<RigidBodyBasics, YoFrameVector3D>();
   private final Map<RigidBodyBasics, YoFramePoint3D> pointsOfApplication = new LinkedHashMap<RigidBodyBasics, YoFramePoint3D>();
   private final Map<RigidBodyBasics, YoGraphicVector> forceVisualizers = new LinkedHashMap<RigidBodyBasics, YoGraphicVector>();
   private final Map<RigidBodyBasics, YoGraphicVector> torqueVisualizers = new LinkedHashMap<RigidBodyBasics, YoGraphicVector>();

   private final Wrench tempWrench = new Wrench();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final ArrayList<RigidBodyBasics> rigidBodies = new ArrayList<RigidBodyBasics>();
   
   public static WrenchVisualizer createWrenchVisualizerWithContactableBodies(String name, List<? extends ContactableBody> contactableBodies, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoRegistry parentRegistry)
   {
      return new WrenchVisualizer(name, extractRigidBodyList(contactableBodies), vizScaling, yoGraphicsListRegistry, parentRegistry);
   }

   public WrenchVisualizer(String name, List<RigidBodyBasics> rigidBodies, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoRegistry parentRegistry)
   {
      this(name, rigidBodies, vizScaling, yoGraphicsListRegistry, parentRegistry, YoAppearance.OrangeRed(), YoAppearance.CornflowerBlue());
   }

   public WrenchVisualizer(String name, List<RigidBodyBasics> rigidBodies, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoRegistry parentRegistry, AppearanceDefinition forceAppearance, AppearanceDefinition torqueAppearance)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);

      this.rigidBodies.addAll(rigidBodies);
      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         String prefix = name + rigidBody.getName();
         YoFrameVector3D force = new YoFrameVector3D(prefix + "Force", ReferenceFrame.getWorldFrame(), registry);
         forces.put(rigidBody, force);

         YoFrameVector3D torque = new YoFrameVector3D(prefix + "Torque", ReferenceFrame.getWorldFrame(), registry);
         torques.put(rigidBody, torque);

         YoFramePoint3D pointOfApplication = new YoFramePoint3D(prefix + "PointOfApplication", ReferenceFrame.getWorldFrame(), registry);
         pointsOfApplication.put(rigidBody, pointOfApplication);

         YoGraphicVector forceVisualizer = new YoGraphicVector(prefix + "ForceViz", pointOfApplication, force, FORCE_VECTOR_SCALE * vizScaling, forceAppearance,
               true);
         forceVisualizers.put(rigidBody, forceVisualizer);
         yoGraphicsList.add(forceVisualizer);

         YoGraphicVector torqueVisualizer = new YoGraphicVector(prefix + "TorqueViz", pointOfApplication, torque, TORQUE_VECTOR_SCALE * vizScaling,
               torqueAppearance, true);
         torqueVisualizers.put(rigidBody, torqueVisualizer);
         yoGraphicsList.add(torqueVisualizer);
      }

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      parentRegistry.addChild(registry);
   }

   public void visualize(Map<RigidBodyBasics, ? extends WrenchReadOnly> wrenches)
   {
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodies.get(i);
         WrenchReadOnly wrench;
         if ((wrench = wrenches.get(rigidBody)) != null)
         {
            tempWrench.setIncludingFrame(wrench);
            tempWrench.changeFrame(tempWrench.getBodyFrame());

            YoFrameVector3D force = forces.get(rigidBody);
            tempVector.setToZero(tempWrench.getReferenceFrame());
            tempVector.set(tempWrench.getLinearPart());
            tempVector.changeFrame(ReferenceFrame.getWorldFrame());
            force.set(tempVector);

            YoFrameVector3D torque = torques.get(rigidBody);
            tempVector.setToZero(tempWrench.getReferenceFrame());
            tempVector.set(tempWrench.getAngularPart());
            tempVector.changeFrame(ReferenceFrame.getWorldFrame());
            torque.set(tempVector);

            YoFramePoint3D pointOfApplication = pointsOfApplication.get(rigidBody);
            tempPoint.setToZero(wrench.getBodyFrame());
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            pointOfApplication.set(tempPoint);
         }
         else
         {
            forces.get(rigidBody).set(Double.NaN, Double.NaN, Double.NaN);
            torques.get(rigidBody).set(Double.NaN, Double.NaN, Double.NaN);
            pointsOfApplication.get(rigidBody).set(Double.NaN, Double.NaN, Double.NaN);
         }
      }
   }

   private static List<RigidBodyBasics> extractRigidBodyList(List<? extends ContactableBody> contactableBodies)
   {
      List<RigidBodyBasics> ret = new ArrayList<>(contactableBodies.size());
      for (int i = 0; i < contactableBodies.size(); i++)
         ret.add(contactableBodies.get(i).getRigidBody());
      return ret;
   }
}

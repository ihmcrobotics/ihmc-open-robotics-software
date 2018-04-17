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
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 6/2/13
 */
public class WrenchVisualizer
{
   private static final double FORCE_VECTOR_SCALE = 0.0015;
   private static final double TORQUE_VECTOR_SCALE = 0.0015;

   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final Map<RigidBody, YoFrameVector3D> forces = new LinkedHashMap<RigidBody, YoFrameVector3D>();
   private final Map<RigidBody, YoFrameVector3D> torques = new LinkedHashMap<RigidBody, YoFrameVector3D>();
   private final Map<RigidBody, YoFramePoint3D> pointsOfApplication = new LinkedHashMap<RigidBody, YoFramePoint3D>();
   private final Map<RigidBody, YoGraphicVector> forceVisualizers = new LinkedHashMap<RigidBody, YoGraphicVector>();
   private final Map<RigidBody, YoGraphicVector> torqueVisualizers = new LinkedHashMap<RigidBody, YoGraphicVector>();

   private final Wrench tempWrench = new Wrench();
   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPoint = new FramePoint3D();
   private final ArrayList<RigidBody> rigidBodies = new ArrayList<RigidBody>();
   
   public static WrenchVisualizer createWrenchVisualizerWithContactableBodies(String name, List<? extends ContactableBody> contactableBodies, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      return new WrenchVisualizer(name, extractRigidBodyList(contactableBodies), vizScaling, yoGraphicsListRegistry, parentRegistry);
   }

   public WrenchVisualizer(String name, List<RigidBody> rigidBodies, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this(name, rigidBodies, vizScaling, yoGraphicsListRegistry, parentRegistry, YoAppearance.OrangeRed(), YoAppearance.CornflowerBlue());
   }

   public WrenchVisualizer(String name, List<RigidBody> rigidBodies, double vizScaling, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry, AppearanceDefinition forceAppearance, AppearanceDefinition torqueAppearance)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(name);

      this.rigidBodies.addAll(rigidBodies);
      for (RigidBody rigidBody : rigidBodies)
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

   public void visualize(Map<RigidBody, Wrench> wrenches)
   {
      for (int i = 0; i < rigidBodies.size(); i++)
      {
         RigidBody rigidBody = rigidBodies.get(i);
         Wrench wrench;
         if ((wrench = wrenches.get(rigidBody)) != null)
         {
            tempWrench.set(wrench);
            tempWrench.changeFrame(tempWrench.getBodyFrame());

            YoFrameVector3D force = forces.get(rigidBody);
            tempVector.setToZero(tempWrench.getExpressedInFrame());
            tempWrench.getLinearPart(tempVector);
            tempVector.changeFrame(ReferenceFrame.getWorldFrame());
            force.set(tempVector);

            YoFrameVector3D torque = torques.get(rigidBody);
            tempVector.setToZero(tempWrench.getExpressedInFrame());
            tempWrench.getAngularPart(tempVector);
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

   private static List<RigidBody> extractRigidBodyList(List<? extends ContactableBody> contactableBodies)
   {
      List<RigidBody> ret = new ArrayList<>(contactableBodies.size());
      for (int i = 0; i < contactableBodies.size(); i++)
         ret.add(contactableBodies.get(i).getRigidBody());
      return ret;
   }
}

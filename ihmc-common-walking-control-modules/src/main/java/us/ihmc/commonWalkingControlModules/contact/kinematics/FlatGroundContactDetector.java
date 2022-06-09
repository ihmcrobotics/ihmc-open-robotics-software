package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class FlatGroundContactDetector implements Updatable
{
   private static final double defaultContactThreshold = 0.03;

   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   protected StatusMessageOutputManager statusOutputManager;
   protected YoBoolean useAbsoluteGroundLocation = new YoBoolean("useAbsoluteGroundHeight", registry);
   protected YoDouble groundHeight = new YoDouble("groundHeight", registry);
   protected YoDouble contactThreshold = new YoDouble("contactThreshold", registry);

   protected final List<RigidBodyBasics> contactableRigidBodies = new ArrayList<>();
   protected final List<ContactableRigidBodyDescription> allCollidables = new ArrayList<>();
   protected final Map<RigidBodyBasics, List<ContactableRigidBodyDescription>> contactableRigidBodyCollidables = new HashMap<>();

   protected final Map<RigidBodyBasics, List<FramePoint3DReadOnly>> contactPointMap = new HashMap<>();
   protected final Map<RigidBodyBasics, FrameVector3DReadOnly> contactNormalMap = new HashMap<>(); // down the road consider different normals per rigid body
   protected final Map<RigidBodyBasics, List<ContactPointVisualization>> contactPointVisualizations = new HashMap<>();

   public FlatGroundContactDetector(RigidBodyBasics rootBody,
                                    RobotCollisionModel collisionModel,
                                    YoGraphicsListRegistry graphicsListRegistry,
                                    List<String> collidableRigidBodies)
   {
      contactThreshold.set(defaultContactThreshold);

      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(rootBody);
      Map<RigidBodyBasics, List<Collidable>> allCollidableMap = robotCollidables.stream().collect(Collectors.groupingBy(Collidable::getRigidBody));
      List<? extends RigidBodyBasics> allRigidBodies = rootBody.subtreeList();

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = allRigidBodies.get(i);
         if (!collidableRigidBodies.contains(rigidBody.getName()))
            continue;

         contactableRigidBodies.add(rigidBody);
         List<Collidable> collidables = allCollidableMap.get(rigidBody);
         List<ContactableRigidBodyDescription> rigidBodyCollidables = new ArrayList<>();
         int numberOfBoxes = 0;

         for (int j = 0; j < collidables.size(); j++)
         {
            Collidable collidable = collidables.get(j);
            if (collidable.getShape() instanceof FrameBox3DReadOnly)
            {
               rigidBodyCollidables.add(new ContactableBoxDescription(rigidBody.getName() + numberOfBoxes++, collidable));
            }
            else
            {
               rigidBodyCollidables.add(new ContactableShapeDescription(collidable));
            }
         }

         allCollidables.addAll(rigidBodyCollidables);
         contactableRigidBodyCollidables.put(rigidBody, rigidBodyCollidables);

         List<ContactPointVisualization> contactPointVisualizations = new ArrayList<>();
         for (int j = 0; j < 4 * collidables.size(); j++)
         {
            ContactPointVisualization contactPointVisualization = new ContactPointVisualization(registry, graphicsListRegistry);
            contactPointVisualization.setVerticalContactNormal();
            contactPointVisualizations.add(contactPointVisualization);
         }

         this.contactPointVisualizations.put(rigidBody, contactPointVisualizations);
      }
   }

   @Override
   public void update(double time)
   {
      double groundHeight = allCollidables.stream().mapToDouble(ContactableRigidBodyDescription::updateHeightInWorld).min().getAsDouble();
      if (useAbsoluteGroundLocation.getValue())
      {
         groundHeight = this.groundHeight.getValue();
      }
      double threshold = groundHeight + contactThreshold.getValue();

      reset();

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         List<ContactableRigidBodyDescription> rigidBodyCollidables = contactableRigidBodyCollidables.get(contactableRigidBodies.get(i));
         List<ContactPointVisualization> visualization = contactPointVisualizations.get(contactableRigidBodies.get(i));

         List<FramePoint3DReadOnly> contactPoints = new ArrayList<>();
         for (int j = 0; j < rigidBodyCollidables.size(); j++)
         {
            rigidBodyCollidables.get(j).packContactPoints(contactPoints, threshold);
         }

         for (int j = 0; j < contactPoints.size(); j++)
         {
            visualization.get(j).getContactPointPosition().set(contactPoints.get(j));
         }
      }
   }

   protected void reset()
   {
      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactableRigidBodies.get(i);
         List<ContactPointVisualization> visualization = contactPointVisualizations.get(rigidBody);
         for (int j = 0; j < visualization.size(); j++)
         {
            visualization.get(j).hide();
         }
      }
   }

   public void setUseAbsoluteGroundLocation(boolean useAbsoluteGroundLocation)
   {
      this.useAbsoluteGroundLocation.set(useAbsoluteGroundLocation);
   }

   public void setGroundHeight(double groundHeight)
   {
      this.groundHeight.set(groundHeight);
   }

   public void setStatusOutputManager(StatusMessageOutputManager statusOutputManager)
   {
      this.statusOutputManager = statusOutputManager;
   }

   public void setContactThreshold(double contactThreshold)
   {
      this.contactThreshold.set(contactThreshold);
   }
}

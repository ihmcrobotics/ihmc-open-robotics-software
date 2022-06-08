package us.ihmc.commonWalkingControlModules.contact.geometry;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class KinematicsFlatGroundContactDetector
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FullHumanoidRobotModel fullRobotModel;
   private final StatusMessageOutputManager statusOutputManager;

   private final List<RigidBodyBasics> contactableRigidBodies = new ArrayList<>();
   private final List<RigidBodyCollisionDescription> allCollidables = new ArrayList<>();
   private final Map<RigidBodyBasics, List<RigidBodyCollisionDescription>> contactableRigidBodyCollidables = new HashMap<>();

   private final Map<RigidBodyBasics, List<FramePoint3DReadOnly>> contactPointMap = new HashMap<>();
   private final Map<RigidBodyBasics, FrameVector3DReadOnly> contactNormalMap = new HashMap<>(); // down the road consider different normals per rigid body
   private final Map<RigidBodyBasics, List<ContactPointVisualization>> contactPointVisualizations = new HashMap<>();

   public KinematicsFlatGroundContactDetector(StatusMessageOutputManager statusOutputManager,
                                              HighLevelHumanoidControllerToolbox highLevelControllerToolbox,
                                              RobotCollisionModel collisionModel,
                                              List<String> collidableRigidBodies)
   {
      this.statusOutputManager = statusOutputManager;
      this.fullRobotModel = highLevelControllerToolbox.getFullRobotModel();

      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(fullRobotModel.getRootBody());
      Map<RigidBodyBasics, List<Collidable>> allCollidableMap = robotCollidables.stream().collect(Collectors.groupingBy(Collidable::getRigidBody));
      List<? extends RigidBodyBasics> allRigidBodies = fullRobotModel.getRootBody().subtreeList();

      for (int i = 0; i < allRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = allRigidBodies.get(i);
         if (!collidableRigidBodies.contains(rigidBody.getName()))
            continue;

         contactableRigidBodies.add(rigidBody);
         List<Collidable> collidables = allCollidableMap.get(rigidBody);
         List<RigidBodyCollisionDescription> rigidBodyCollidables = new ArrayList<>();
         int numberOfBoxes = 0;

         for (int j = 0; j < collidables.size(); j++)
         {
            Collidable collidable = collidables.get(j);
            if (collidable.getShape() instanceof FrameBox3DReadOnly)
            {
               rigidBodyCollidables.add(new BoxCollisionDescription(rigidBody.getName() + numberOfBoxes++, collidable));
            }
            else
            {
               rigidBodyCollidables.add(new ShapeCollisionDescription(collidable));
            }
         }

         allCollidables.addAll(rigidBodyCollidables);
         contactableRigidBodyCollidables.put(rigidBody, rigidBodyCollidables);

         List<ContactPointVisualization> contactPointVisualizations = new ArrayList<>();
         for (int j = 0; j < 4 * collidables.size(); j++)
         {
            contactPointVisualizations.add(new ContactPointVisualization(registry, highLevelControllerToolbox.getYoGraphicsListRegistry()));
         }
         this.contactPointVisualizations.put(rigidBody, contactPointVisualizations);
      }
   }

   public void update()
   {
      double minimumHeight = allCollidables.stream().mapToDouble(RigidBodyCollisionDescription::updateHeightInWorld).min().getAsDouble();
      double threshold = minimumHeight + 0.03;

      for (int i = 0; i < contactableRigidBodies.size(); i++)
      {
         RigidBodyBasics rigidBody = contactableRigidBodies.get(i);
         List<RigidBodyCollisionDescription> rigidBodyCollidables = contactableRigidBodyCollidables.get(rigidBody);

         List<ContactPointVisualization> visualization = contactPointVisualizations.get(rigidBody);
         for (int j = 0; j < visualization.size(); j++)
         {
            visualization.get(j).hide();
         }

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

   private static int visualizerIndex = 0;

   private static class ContactPointVisualization
   {
      private final YoFramePoint3D contactPointPosition;
      private final YoFrameVector3D contactPointNormal;

      public ContactPointVisualization(YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
      {
         int index = visualizerIndex++;
         contactPointPosition = new YoFramePoint3D("contactPoint" + index, ReferenceFrame.getWorldFrame(), registry);
         contactPointNormal = new YoFrameVector3D("contactNormal" + index, ReferenceFrame.getWorldFrame(), registry);

         contactPointNormal.setZ(1.0);

         YoGraphicPosition contactPointGraphic = new YoGraphicPosition("contactPointGraphic" + index, contactPointPosition, 0.01, YoAppearance.Red());
         YoGraphicVector contactNormalGraphic = new YoGraphicVector("contactPointVector" + index, contactPointPosition, contactPointNormal, 0.1, YoAppearance.Red());
         graphicsListRegistry.registerYoGraphic("Contact Points", contactPointGraphic);
         graphicsListRegistry.registerYoGraphic("Contact Points", contactNormalGraphic);
      }

      public void update(Tuple3DReadOnly position, Tuple3DReadOnly normal)
      {
         contactPointPosition.set(position);
         contactPointNormal.set(normal);
      }

      public void hide()
      {
         contactPointPosition.setToNaN();
      }

      public YoFramePoint3D getContactPointPosition()
      {
         return contactPointPosition;
      }

      public YoFrameVector3D getContactPointNormal()
      {
         return contactPointNormal;
      }
   }
}

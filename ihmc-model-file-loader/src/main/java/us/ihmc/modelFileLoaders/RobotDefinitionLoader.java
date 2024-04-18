package us.ihmc.modelFileLoaders;

import java.io.InputStream;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.function.Consumer;
import java.util.function.Predicate;

import javax.xml.bind.JAXBException;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;

public class RobotDefinitionLoader
{
   public static final String DEFAULT_ROOT_BODY_NAME = "elevator";

   public static RobotDefinition loadURDFModel(InputStream inputStream,
                                               Collection<String> resourceDirectories,
                                               ClassLoader classLoader,
                                               String modelName,
                                               ContactPointDefinitionHolder contactPointDefinitionHolder,
                                               JointNameMap<?> jointNameMap,
                                               boolean removeCollisionMeshes)
   {
      return loadURDFModel(inputStream,
                           resourceDirectories,
                           classLoader,
                           modelName,
                           contactPointDefinitionHolder,
                           jointNameMap,
                           removeCollisionMeshes,
                           URDFTools.DEFAULT_URDF_PARSER_PROPERTIES);
   }

   public static RobotDefinition loadURDFModel(InputStream inputStream,
                                               Collection<String> resourceDirectories,
                                               ClassLoader classLoader,
                                               String modelName,
                                               ContactPointDefinitionHolder contactPointDefinitionHolder,
                                               JointNameMap<?> jointNameMap,
                                               boolean removeCollisionMeshes,
                                               URDFTools.URDFParserProperties urdfParserProperties)
   {
      try
      {
         URDFModel urdfRoot = URDFTools.loadURDFModel(inputStream, resourceDirectories, classLoader);
         RobotDefinition robotDefinition = URDFTools.toRobotDefinition(urdfRoot, urdfParserProperties);
         // By default SDFTools names the root body "rootBody", for backward compatibility it is renamed "elevator".
         robotDefinition.getRootBodyDefinition().setName(DEFAULT_ROOT_BODY_NAME);

         if (contactPointDefinitionHolder != null)
            addGroundContactPoints(robotDefinition, contactPointDefinitionHolder);

         if (jointNameMap != null)
         {
            for (String jointName : jointNameMap.getLastSimulatedJoints())
               robotDefinition.addSubtreeJointsToIgnore(jointName);
            adjustJointLimitStops(robotDefinition, jointNameMap);
         }
         adjustRigidBodyInterias(robotDefinition);

         if (removeCollisionMeshes)
            removeCollisionShapeDefinitions(robotDefinition);

         return robotDefinition;
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static RobotDefinition loadSDFModel(InputStream stream,
                                              Collection<String> resourceDirectories,
                                              ClassLoader classLoader,
                                              String modelName,
                                              ContactPointDefinitionHolder contactPointDefinitionHolder,
                                              JointNameMap<?> jointNameMap,
                                              boolean removeCollisionMeshes)
   {
      try
      {
         SDFRoot sdfRoot = SDFTools.loadSDFRoot(stream, resourceDirectories, classLoader);
         RobotDefinition robotDefinition = SDFTools.toFloatingRobotDefinition(sdfRoot, modelName);
         // By default SDFTools names the root body "rootBody", for backward compatibility it is renamed "elevator".
         robotDefinition.getRootBodyDefinition().setName(DEFAULT_ROOT_BODY_NAME);

         if (contactPointDefinitionHolder != null)
            addGroundContactPoints(robotDefinition, contactPointDefinitionHolder);

         if (jointNameMap != null)
         {
            for (String jointName : jointNameMap.getLastSimulatedJoints())
               robotDefinition.addSubtreeJointsToIgnore(jointName);
            adjustJointLimitStops(robotDefinition, jointNameMap);
         }
         adjustRigidBodyInterias(robotDefinition);

         if (removeCollisionMeshes)
            removeCollisionShapeDefinitions(robotDefinition);

         return robotDefinition;
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition)
   {
      setDefaultMaterial(robotDefinition, new MaterialDefinition(ColorDefinitions.Orange().derive(0, 1, 1, 0.4)));
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition, MaterialDefinition defaultMaterial)
   {
      for (RigidBodyDefinition rigidBodyDefinition : robotDefinition.getAllRigidBodies())
      {
         for (VisualDefinition visualDefinition : rigidBodyDefinition.getVisualDefinitions())
         {
            if (visualDefinition.getMaterialDefinition() != null)
               continue;
            GeometryDefinition geometryDefinition = visualDefinition.getGeometryDefinition();
            if (geometryDefinition instanceof ModelFileGeometryDefinition)
            {
               if (!((ModelFileGeometryDefinition) geometryDefinition).getFileName().toLowerCase().endsWith(".stl"))
                  continue;
            }
            visualDefinition.setMaterialDefinition(defaultMaterial);
         }
      }
   }

   public static void removeCollisionShapeDefinitions(RobotDefinition robotDefinition)
   {
      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
         body.getCollisionShapeDefinitions().clear();
   }

   public static void setRobotDefinitionMaterial(RobotDefinition robotDefinition, MaterialDefinition materialDefinition)
   {
      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
      {
         body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(materialDefinition));
      }
   }

   public static void setRobotDefinitionTransparency(RobotDefinition robotDefinition, double transparency)
   {
      setRobotDefinitionMaterial(robotDefinition, new MaterialDefinition(ColorDefinitions.Orange().derive(0, 1, 1, 1.0 - transparency)));
   }

   public static void adjustJointLimitStops(RobotDefinition robotDefinition, JointNameMap<?> jointNameMap)
   {
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (joint instanceof RevoluteJointDefinition)
         {
            RevoluteJointDefinition revoluteJoint = (RevoluteJointDefinition) joint;

            if (isJointInNeedOfReducedGains(joint.getName()))
            {
               revoluteJoint.setKpSoftLimitStop(10.0);
               revoluteJoint.setKdSoftLimitStop(2.5);
            }
            else if (revoluteJoint.getKpSoftLimitStop() <= 0.0 && revoluteJoint.getKdSoftLimitStop() <= 0.0)
            {
               revoluteJoint.setKpSoftLimitStop(jointNameMap.getJointKLimit(joint.getName()));
               revoluteJoint.setKdSoftLimitStop(jointNameMap.getJointBLimit(joint.getName()));
            }
            else
            {
               revoluteJoint.setKpSoftLimitStop(0.0001 * revoluteJoint.getKpSoftLimitStop());
               revoluteJoint.setKdSoftLimitStop(0.1 * revoluteJoint.getKdSoftLimitStop());
            }

            if (!isJointInNeedOfReducedGains(joint.getName()))
            {
               revoluteJoint.setDampingVelocitySoftLimit(jointNameMap.getDefaultVelocityLimitDamping());
            }
         }
         else if (joint instanceof PrismaticJointDefinition)
         {
            PrismaticJointDefinition prismaticJoint = (PrismaticJointDefinition) joint;

            if (isJointInNeedOfReducedGains(joint.getName()))
            {
               prismaticJoint.setKpSoftLimitStop(100.0);
               prismaticJoint.setKdSoftLimitStop(20.0);
            }
            else if (prismaticJoint.getKpSoftLimitStop() <= 0.0 && prismaticJoint.getKdSoftLimitStop() <= 0.0)
            {
               prismaticJoint.setKpSoftLimitStop(jointNameMap.getJointKLimit(joint.getName()));
               prismaticJoint.setKdSoftLimitStop(jointNameMap.getJointBLimit(joint.getName()));
            }
            else
            {
               prismaticJoint.setKpSoftLimitStop(0.0001 * prismaticJoint.getKpSoftLimitStop());
               prismaticJoint.setKdSoftLimitStop(prismaticJoint.getKdSoftLimitStop());
            }

            if (!isJointInNeedOfReducedGains(joint.getName()))
            {
               prismaticJoint.setDampingVelocitySoftLimit(jointNameMap.getDefaultVelocityLimitDamping());
            }
         }
      }
   }

   public static void adjustRigidBodyInterias(RobotDefinition robotDefinition)
   {
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (isJointInNeedOfReducedGains(joint.getName()))
         {
            RigidBodyDefinition successor = joint.getSuccessor();
            successor.getMomentOfInertia().scale(100.0);
         }
      }
   }

   private static boolean isJointInNeedOfReducedGains(String jointName)
   {
      return jointName.contains("f0") || jointName.contains("f1") || jointName.contains("f2") || jointName.contains("f3") || jointName.contains("palm")
            || jointName.contains("finger");
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder)
   {
      addGroundContactPoints(robotDefinition, contactPointHolder, true);
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder, boolean addVisualization)
   {
      addGroundContactPoints(robotDefinition, contactPointHolder, addVisualization ? 0.01 : 0.0);
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder, double contactPointVizSize)
   {
      if (contactPointHolder == null)
         return;

      LinkedHashMap<String, Integer> counters = new LinkedHashMap<String, Integer>();
      for (ImmutablePair<String, Vector3D> jointContactPoint : contactPointHolder.getJointNameGroundContactPointMap())
      {
         String jointName = jointContactPoint.getLeft();

         int count;
         if (counters.get(jointName) == null)
            count = 0;
         else
            count = counters.get(jointName);

         Vector3D gcOffset = jointContactPoint.getRight();

         GroundContactPointDefinition groundContactPoint = new GroundContactPointDefinition();
         groundContactPoint.setName("gc_" + jointName + "_" + count++);
         groundContactPoint.getTransformToParent().getTranslation().set(gcOffset);
         groundContactPoint.setGroupIdentifier(contactPointHolder.getGroupIdentifier(jointContactPoint));
         ExternalWrenchPointDefinition externalWrenchPoint = new ExternalWrenchPointDefinition();
         externalWrenchPoint.setName("ef_" + jointName + "_" + count++);
         externalWrenchPoint.getTransformToParent().getTranslation().set(gcOffset);

         JointDefinition jointDefinition = robotDefinition.getJointDefinition(jointName);

         if (jointDefinition == null) // In the case there are no joints of that name
         {
            LogTools.error("No joint named {}. Skipping...", jointName);
            continue;
         }

         jointDefinition.addGroundContactPointDefinition(groundContactPoint);
         jointDefinition.addExternalWrenchPointDefinition(externalWrenchPoint);

         counters.put(jointName, count);

         if (Double.isFinite(contactPointVizSize) && contactPointVizSize > 0.0)
         {
            VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
            visualDefinitionFactory.appendTranslation(jointContactPoint.getRight());
            visualDefinitionFactory.addSphere(contactPointVizSize, ColorDefinitions.Orange());
            jointDefinition.getSuccessor().getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());
         }
      }
   }

   public static void scaleRobotDefinition(RobotDefinition definition, double modelScale, double massScalePower, Predicate<JointDefinition> jointFilter)
   {
      scaleRigidBodyDefinitionRecursive(definition.getRootBodyDefinition(), modelScale, massScalePower, jointFilter, true);
   }

   public static void scaleRigidBodyDefinitionRecursive(RigidBodyDefinition definition,
                                                        double modelScale,
                                                        double massScalePower,
                                                        Predicate<JointDefinition> jointFilter,
                                                        boolean scaleInertia)
   {
      if (scaleInertia && definition.getParentJoint() != null)
         scaleInertia &= jointFilter.test(definition.getParentJoint());

      scaleRigidBodyDefinition(definition, modelScale, massScalePower, scaleInertia);

      for (JointDefinition joint : definition.getChildrenJoints())
      {
         scaleJointDefinition(joint, modelScale, massScalePower);
         scaleRigidBodyDefinitionRecursive(joint.getSuccessor(), modelScale, massScalePower, jointFilter, scaleInertia);
      }
   }

   public static void scaleJointDefinition(JointDefinition definition, double modelScale, double massScalePower)
   {
      definition.getTransformToParent().getTranslation().scale(modelScale);

      for (SensorDefinition sensor : definition.getSensorDefinitions())
         sensor.getTransformToJoint().getTranslation().scale(modelScale);
      for (KinematicPointDefinition kp : definition.getKinematicPointDefinitions())
         kp.getTransformToParent().getTranslation().scale(modelScale);
      // TODO This seems inconsistent, but that's how we used to do it when using RobotDescription.
      //      for (ExternalWrenchPointDefinition efp : definition.getExternalWrenchPointDefinitions())
      //         efp.getTransformToParent().getTranslation().scale(modelScale);
      //      for (GroundContactPointDefinition gcp : definition.getGroundContactPointDefinitions())
      //         gcp.getTransformToParent().getTranslation().scale(modelScale);

      if (definition instanceof OneDoFJointDefinition)
      {
         double massScale = Math.pow(modelScale, massScalePower);
         OneDoFJointDefinition oneDoFJoint = (OneDoFJointDefinition) definition;
         oneDoFJoint.setDamping(massScale * oneDoFJoint.getDamping());
         oneDoFJoint.setKpSoftLimitStop(massScale * oneDoFJoint.getKpSoftLimitStop());
         oneDoFJoint.setKdSoftLimitStop(massScale * oneDoFJoint.getKdSoftLimitStop());
         oneDoFJoint.setDampingVelocitySoftLimit(massScale * oneDoFJoint.getDampingVelocitySoftLimit());
      }
   }

   public static void scaleRigidBodyDefinition(RigidBodyDefinition definition, double modelScale, double massScalePower, boolean scaleInertia)
   {
      // Center of mass offset scales with the scaling factor
      definition.getCenterOfMassOffset().scale(modelScale);

      // Mass scales with factor^massScalePower. massScalePower is 3 when considering constant density

      if (scaleInertia)
      {
         double massScale = Math.pow(modelScale, massScalePower);
         definition.setMass(massScale * definition.getMass());

         if (definition.getMomentOfInertia() != null)
         {
            // The components of the inertia matrix are defined with int(r^2 dm). So they scale factor ^ (2 + massScalePower)
            double inertiaScale = Math.pow(modelScale, massScalePower + 2);
            definition.getMomentOfInertia().scale(inertiaScale);
         }
      }

      for (VisualDefinition visual : definition.getVisualDefinitions())
         visual.getOriginPose().appendScale(modelScale);
      for (CollisionShapeDefinition collision : definition.getCollisionShapeDefinitions())
         scaleCollisionShapeDefinition(collision, modelScale);
   }

   private static void scaleCollisionShapeDefinition(CollisionShapeDefinition definition, double scale)
   {
      throw new RuntimeException("TODO: Implement me");
   }

   public static Consumer<RobotDefinition> jointLimitRemover()
   {
      return jointLimitRemover(null);
   }

   public static Consumer<RobotDefinition> jointLimitRemover(String nameFilter)
   {
      return jointLimitMutator(nameFilter, -100.0, 100.0);
   }

   public static Consumer<RobotDefinition> jointLimitMutator(String nameFilter, double lowerLimit, double upperLimit)
   {
      return robotDefinition ->
      {
         for (JointDefinition joint : robotDefinition.getAllJoints())
         {
            String jointName = joint.getName();
            if (!(joint instanceof OneDoFJointDefinition))
               continue;
            if (nameFilter != null && !nameFilter.isEmpty() && !jointName.contains(nameFilter))
               continue;

            ((OneDoFJointDefinition) joint).setPositionLimits(lowerLimit, upperLimit);
         }
      };
   }
}

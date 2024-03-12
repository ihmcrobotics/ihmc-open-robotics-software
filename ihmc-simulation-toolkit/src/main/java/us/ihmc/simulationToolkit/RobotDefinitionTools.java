package us.ihmc.simulationToolkit;

import java.io.InputStream;
import java.util.Collection;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.color.MutableColor;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.robotModels.description.RobotDefinitionConverter;
import us.ihmc.robotModels.description.RobotDescriptionConverter;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateBasics;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationConstructionSetTools.tools.TerrainObjectDefinitionTools;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotDefinitionTools
{
   public static void setRobotInitialState(RobotDefinition definition, Robot robot)
   {
      definition.forEachJointDefinition(jointDefinition ->
      {
         if (jointDefinition.getInitialJointState() == null)
            return;
         Joint joint = robot.getJoint(jointDefinition.getName());
         if (joint == null)
         {
            LogTools.error("Could not find joint {} during initialization.", jointDefinition.getName());
            return;
         }

         if (joint instanceof FloatingJoint)
         {
            SixDoFJointStateBasics initialState = (SixDoFJointStateBasics) jointDefinition.getInitialJointState();
            FloatingJoint floatingJoint = (FloatingJoint) joint;
            if (initialState.hasOutputFor(JointStateType.CONFIGURATION))
            {
               floatingJoint.setPosition(initialState.getPosition());
               floatingJoint.setOrientation(initialState.getOrientation());
            }
            if (initialState.hasOutputFor(JointStateType.VELOCITY))
            {
               floatingJoint.setAngularVelocityInBody(initialState.getLinearVelocity());
               Vector3D linearVelocity = new Vector3D();
               floatingJoint.getOrientation().transform(initialState.getLinearVelocity(), linearVelocity);
               floatingJoint.setVelocity(linearVelocity);
            }
            if (initialState.hasOutputFor(JointStateType.ACCELERATION))
               LogTools.warn("Acceleration is not supported.");
            if (initialState.hasOutputFor(JointStateType.EFFORT))
               LogTools.warn("Effort is not supported.");
         }
         else if (joint instanceof OneDegreeOfFreedomJoint)
         {
            OneDoFJointStateBasics initialState = (OneDoFJointStateBasics) jointDefinition.getInitialJointState();
            OneDegreeOfFreedomJoint oneDoFJoint = (OneDegreeOfFreedomJoint) joint;
            if (initialState.hasOutputFor(JointStateType.CONFIGURATION))
               oneDoFJoint.setQ(initialState.getConfiguration());
            if (initialState.hasOutputFor(JointStateType.VELOCITY))
               oneDoFJoint.setQ(initialState.getVelocity());
            if (initialState.hasOutputFor(JointStateType.ACCELERATION))
               oneDoFJoint.setQ(initialState.getAcceleration());
            if (initialState.hasOutputFor(JointStateType.EFFORT))
               oneDoFJoint.setQ(initialState.getEffort());
         }
         else
         {
            LogTools.warn("Joint type not supported for initialization: {} ", joint.getClass().getSimpleName());
         }
      });
   }

   public static void addCollisionsToRobotDefinition(List<Collidable> collidables, RobotDefinition robotDefinition)
   {
      for (Collidable collidable : collidables)
      {
         if (collidable.getRigidBody() == null)
            continue;

         RigidBodyDefinition rigidBodyDefinition = robotDefinition.getRigidBodyDefinition(collidable.getRigidBody().getName());
         rigidBodyDefinition.addCollisionShapeDefinition(TerrainObjectDefinitionTools.toCollisionShapeDefinition(collidable));
      }
   }

   public static void addInitialStateToRobotDefinition(Robot robotAtInitialState, RobotDefinition robotDefinition)
   {
      for (Joint rootJoint : robotAtInitialState.getRootJoints())
      {
         JointDefinition rootJointDefinition = robotDefinition.getRootJointDefinitions().stream().filter(j -> j.getName().equals(rootJoint.getName()))
                                                              .findFirst().get();
         addInitialStateRecursive(rootJoint, rootJointDefinition);
      }
   }

   private static void addInitialStateRecursive(Joint jointAtInitialState, JointDefinition jointDefinition)
   {
      if (jointAtInitialState instanceof FloatingJoint)
      {
         SixDoFJointState initialJointState = new SixDoFJointState();
         initialJointState.setConfiguration(((FloatingJoint) jointAtInitialState).getOrientation(), ((FloatingJoint) jointAtInitialState).getPosition());
         jointDefinition.setInitialJointState(initialJointState);
      }
      else if (jointAtInitialState instanceof OneDegreeOfFreedomJoint)
      {
         OneDoFJointState initialJointState = new OneDoFJointState();
         initialJointState.setConfiguration(((OneDegreeOfFreedomJoint) jointAtInitialState).getQ());
         initialJointState.setVelocity(((OneDegreeOfFreedomJoint) jointAtInitialState).getQD());
         jointDefinition.setInitialJointState(initialJointState);
      }

      for (Joint childJoint : jointAtInitialState.getChildrenJoints())
      {
         JointDefinition childJointDefinition = jointDefinition.getSuccessor().getChildrenJoints().stream()
                                                               .filter(j -> j.getName().equals(childJoint.getName())).findFirst().get();
         addInitialStateRecursive(childJoint, childJointDefinition);
      }
   }

   public static CollisionShapeDefinition toCollisionShapeDefinition(Collidable source)
   {
      return TerrainObjectDefinitionTools.toCollisionShapeDefinition(source);
   }

   /**
    * This is useful to get a detached copy of an arm or a leg, or perhaps a finger.
    * The base can't be moved from world, as there's no elevator and 6 DoF joint.
    */
   public static RobotDefinition cloneLimbOnlyDefinition(RobotDefinition originalRobotDefinition, String rootBodyName, String jointNameToFollow)
   {
      RobotDefinition robotDefinition = new RobotDefinition();

      RigidBodyDefinition clonedRootBody = originalRobotDefinition.getRigidBodyDefinition(rootBodyName).copyRecursive();
      JointDefinition clonedJointToFollow = originalRobotDefinition.getJointDefinition(jointNameToFollow).copyRecursive();

      clonedRootBody.addChildJoint(clonedJointToFollow);

      robotDefinition.setRootBodyDefinition(clonedRootBody);
      return robotDefinition;
   }

   /**
    * This is useful to get a detached copy of an arm or a leg, or perhaps a finger.
    * The base canbe moved from world, using the elevator and 6 DoF joint.
    */
   public static RobotDefinition cloneLimbOnlyDefinitionWithElevator(RobotDefinition originalRobotDefinition, String rootBodyName, String jointNameToFollow)
   {
      RobotDefinition robotDefinition = new RobotDefinition();

      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition sixDoFJointDefinition = new SixDoFJointDefinition(rootBodyName);
      elevator.addChildJoint(sixDoFJointDefinition);

      RigidBodyDefinition clonedRootBody = originalRobotDefinition.getRigidBodyDefinition(rootBodyName).copyRecursive();
      JointDefinition clonedJointToFollow = originalRobotDefinition.getJointDefinition(jointNameToFollow).copyRecursive();

      clonedRootBody.addChildJoint(clonedJointToFollow);

      sixDoFJointDefinition.setSuccessor(clonedRootBody);

      robotDefinition.setRootBodyDefinition(elevator);
      return robotDefinition;
   }

   // --------------------------------------------------------
   // RobotDefinitionLoader redirections:
   // --------------------------------------------------------

   public static RobotDefinition loadURDFModel(Collection<InputStream> stream,
                                               Collection<String> resourceDirectories,
                                               ClassLoader classLoader,
                                               String modelName,
                                               ContactPointDefinitionHolder contactPointDefinitionHolder,
                                               JointNameMap<?> jointNameMap,
                                               boolean removeCollisionMeshes)
   {
      return RobotDefinitionLoader.loadURDFModel(stream,
                                                 resourceDirectories,
                                                 classLoader,
                                                 modelName,
                                                 contactPointDefinitionHolder,
                                                 jointNameMap,
                                                 removeCollisionMeshes);
   }

   public static RobotDefinition loadSDFModel(InputStream stream,
                                              Collection<String> resourceDirectories,
                                              ClassLoader classLoader,
                                              String modelName,
                                              ContactPointDefinitionHolder contactPointDefinitionHolder,
                                              JointNameMap<?> jointNameMap,
                                              boolean removeCollisionMeshes)
   {
      return RobotDefinitionLoader.loadSDFModel(stream,
                                                resourceDirectories,
                                                classLoader,
                                                modelName,
                                                contactPointDefinitionHolder,
                                                jointNameMap,
                                                removeCollisionMeshes);
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition)
   {
      RobotDefinitionLoader.setDefaultMaterial(robotDefinition);
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition, MaterialDefinition defaultMaterial)
   {
      RobotDefinitionLoader.setDefaultMaterial(robotDefinition, defaultMaterial);
   }

   public static void removeCollisionShapeDefinitions(RobotDefinition robotDefinition)
   {
      RobotDefinitionLoader.removeCollisionShapeDefinitions(robotDefinition);
   }

   public static void setRobotDefinitionMaterial(RobotDefinition robotDefinition, MaterialDefinition materialDefinition)
   {
      RobotDefinitionLoader.setRobotDefinitionMaterial(robotDefinition, materialDefinition);
   }

   public static void setRobotDefinitionTransparency(RobotDefinition robotDefinition, double transparency)
   {
      RobotDefinitionLoader.setRobotDefinitionTransparency(robotDefinition, transparency);
   }

   public static void adjustJointLimitStops(RobotDefinition robotDefinition, JointNameMap<?> jointNameMap)
   {
      RobotDefinitionLoader.adjustJointLimitStops(robotDefinition, jointNameMap);
   }

   public static void adjustRigidBodyInterias(RobotDefinition robotDefinition)
   {
      RobotDefinitionLoader.adjustRigidBodyInterias(robotDefinition);
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder)
   {
      RobotDefinitionLoader.addGroundContactPoints(robotDefinition, contactPointHolder);
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder, boolean addVisualization)
   {
      RobotDefinitionLoader.addGroundContactPoints(robotDefinition, contactPointHolder, addVisualization);
   }

   public static void scaleRobotDefinition(RobotDefinition definition, double modelScale, double massScalePower, Predicate<JointDefinition> jointFilter)
   {
      RobotDefinitionLoader.scaleRobotDefinition(definition, modelScale, massScalePower, jointFilter);
   }

   public static void scaleRigidBodyDefinitionRecursive(RigidBodyDefinition definition,
                                                        double modelScale,
                                                        double massScalePower,
                                                        Predicate<JointDefinition> jointFilter,
                                                        boolean scaleInertia)
   {
      RobotDefinitionLoader.scaleRigidBodyDefinitionRecursive(definition, modelScale, massScalePower, jointFilter, scaleInertia);
   }

   public static void scaleJointDefinition(JointDefinition definition, double modelScale, double massScalePower)
   {
      RobotDefinitionLoader.scaleJointDefinition(definition, modelScale, massScalePower);
   }

   public static void scaleRigidBodyDefinition(RigidBodyDefinition definition, double modelScale, double massScalePower, boolean scaleInertia)
   {
      RobotDefinitionLoader.scaleRigidBodyDefinition(definition, modelScale, massScalePower, scaleInertia);
   }

   public static Consumer<RobotDefinition> jointLimitRemover()
   {
      return RobotDefinitionLoader.jointLimitRemover();
   }

   public static Consumer<RobotDefinition> jointLimitRemover(String nameFilter)
   {
      return RobotDefinitionLoader.jointLimitRemover(nameFilter);
   }

   public static Consumer<RobotDefinition> jointLimitMutator(String nameFilter, double lowerLimit, double upperLimit)
   {
      return RobotDefinitionLoader.jointLimitMutator(nameFilter, lowerLimit, upperLimit);
   }

   // --------------------------------------------------
   // RobotDefinitionConverter redirections:
   // --------------------------------------------------
   public static GraphicsObjectsHolder toGraphicsObjectsHolder(RobotDefinition robotDefinition)
   {
      return RobotDefinitionConverter.toGraphicsObjectsHolder(robotDefinition);
   }

   public static RobotDescription toRobotDescription(RobotDefinition robotDefinition)
   {
      return RobotDefinitionConverter.toRobotDescription(robotDefinition);
   }

   public static Graphics3DObject toGraphics3DObject(Collection<? extends VisualDefinition> source)
   {
      return VisualsConversionTools.toGraphics3DObject(source);
   }

   public static Graphics3DObject toGraphics3DObject(VisualDefinition source)
   {
      return VisualsConversionTools.toGraphics3DObject(source);
   }

   public static List<Graphics3DPrimitiveInstruction> toGraphics3DPrimitiveInstruction(GeometryDefinition source)
   {
      return VisualsConversionTools.toGraphics3DPrimitiveInstruction(source);
   }

   public static AppearanceDefinition toAppearanceDefinition(MaterialDefinition source)
   {
      return VisualsConversionTools.toAppearanceDefinition(source);
   }

   public static List<CollisionMeshDescription> toCollisionMeshDescriptions(Collection<? extends CollisionShapeDefinition> source)
   {
      return RobotDefinitionConverter.toCollisionMeshDescriptions(source);
   }

   public static CollisionMeshDescription toCollisionMeshDescription(CollisionShapeDefinition source)
   {
      return RobotDefinitionConverter.toCollisionMeshDescription(source);
   }

   // --------------------------------------------------
   // RobotDescriptionConverter redirections:
   // --------------------------------------------------

   public static RobotDefinition toRobotDefinition(RobotDescription robotDescription)
   {
      return RobotDescriptionConverter.toRobotDefinition(robotDescription);
   }

   public static List<VisualDefinition> toVisualDefinitions(Graphics3DObject graphics3DObject)
   {
      return VisualsConversionTools.toVisualDefinitions(graphics3DObject);
   }

   public static MaterialDefinition toMaterialDefinition(AppearanceDefinition appearanceDefinition)
   {
      return VisualsConversionTools.toMaterialDefinition(appearanceDefinition);
   }

   public static ColorDefinition toColorDefinition(MutableColor mutableColor, double transparency)
   {
      return VisualsConversionTools.toColorDefinition(mutableColor, transparency);
   }
}

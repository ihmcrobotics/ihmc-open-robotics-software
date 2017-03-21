package us.ihmc.robotDataLogger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.apache.commons.lang3.NotImplementedException;
import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotDataLogger.jointState.JointHolder;
import us.ihmc.robotDataLogger.jointState.JointHolderFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class YoVariableHandShakeBuilder
{
   private final Handshake handshake = new Handshake();
   private final ArrayList<JointHolder> jointHolders = new ArrayList<JointHolder>();   
   private final HashMap<YoVariable<?>, Integer> yoVariableIndices = new HashMap<YoVariable<?>, Integer>();
   private final ArrayList<ImmutablePair<YoVariable<?>, YoVariableRegistry>> variablesAndRootRegistries = new ArrayList<>();

   
   private int registryID = 1;

   public YoVariableHandShakeBuilder(List<RigidBody> rootBodies, double dt)
   {
      createRootRegistry();

      handshake.setDt(dt);

      if (rootBodies != null)
      {
         addJointHolders(rootBodies);
      }
   }

   public void addDynamicGraphicObjects(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArrayList<YoGraphicsList> yoGraphicsLists = new ArrayList<YoGraphicsList>();
      yoGraphicsListRegistry.getRegisteredYoGraphicsLists(yoGraphicsLists);
      for (YoGraphicsList yoGraphicsList : yoGraphicsLists)
      {
         for (YoGraphic yoGraphic : yoGraphicsList.getYoGraphics())
         {
            if (yoGraphic instanceof RemoteYoGraphic)
            {
               DynamicGraphicMessage msg = handshake.getGraphic_object().add();
               msg.setList_name(yoGraphicsList.getLabel());
               messageFromDynamicGraphicObject((RemoteYoGraphic) yoGraphic, msg);
               
            }
            else
            {
               System.err.println("Remote DGO not supported:  " + yoGraphic.getClass().getSimpleName());
            }
         }
      }

      ArrayList<ArtifactList> artifactLists = new ArrayList<ArtifactList>();
      yoGraphicsListRegistry.getRegisteredArtifactLists(artifactLists);

      for (ArtifactList artifactList : artifactLists)
      {
         for (Artifact artifact : artifactList.getArtifacts())
         {
            if (artifact instanceof RemoteYoGraphic)
            {
               DynamicGraphicMessage msg = handshake.getArtifact().add();
               messageFromDynamicGraphicObject((RemoteYoGraphic) artifact, msg);
            }
            else
            {
               System.err.println("Remote artifact not supported: " + artifact.getClass().getSimpleName());
            }
         }
      }
   }

   private void addJointHolders(List<RigidBody> rootBodies)
   {
      InverseDynamicsJoint[] joints = ScrewTools.computeSubtreeJoints(rootBodies);
      for (InverseDynamicsJoint joint : joints)
      {
         JointHolder jointHolder = JointHolderFactory.getJointHolder(joint);

         JointDefinition jointDefinition = handshake.getJoint().add();
         jointDefinition.setName(joint.getName());
         jointDefinition.setType(jointHolder.getJointType());

         jointHolders.add(jointHolder);
      }
   }

   public List<JointHolder> getJointHolders()
   {
      return Collections.unmodifiableList(jointHolders);
   }
   
   public int getNumberOfJointStates()
   {
      return FullStateBuffer.getNumberOfJointStates(jointHolders);
   }

   private void createRootRegistry()
   {
      
      YoRegistryDefinition yoRegistryDescription = handshake.getRegistry().add();
      yoRegistryDescription.setName("main");
      yoRegistryDescription.setParent(0);
   }

   public int addRegistry(YoVariableRegistry registry, List<YoVariable<?>> variableListToPack)
   {
      int offset = variablesAndRootRegistries.size();
      addRegistry(0, registry, variableListToPack, registry);
      return offset;
   }

   private void addRegistry(int parentID, YoVariableRegistry registry, List<YoVariable<?>> variableListToPack, YoVariableRegistry rootRegistry)
   {
      int myID = registryID;
      registryID++;

      YoRegistryDefinition yoRegistryDescription = handshake.getRegistry().add();
      yoRegistryDescription.setName(registry.getName());
      yoRegistryDescription.setParent(parentID);

      addVariables(myID, registry, variableListToPack, rootRegistry);

      for (YoVariableRegistry child : registry.getChildren())
      {
         addRegistry(myID, child, variableListToPack, rootRegistry);
      }

   }

   private void addVariables(int registryID, YoVariableRegistry registry, List<YoVariable<?>> variableListToPack, YoVariableRegistry rootRegistry)
   {
      ArrayList<YoVariable<?>> variables = registry.getAllVariablesInThisListOnly();

      for (YoVariable<?> variable : variables)
      {
         YoVariableDefinition yoVariableDefinition = handshake.getVariable().add();
         yoVariableDefinition.setName(variable.getName());
         yoVariableDefinition.setRegistry(registryID);

         switch (variable.getYoVariableType())
         {
         case DOUBLE:
            yoVariableDefinition.setType(YoType.DoubleYoVariable);
            break;
         case INTEGER:
            yoVariableDefinition.setType(YoType.IntegerYoVariable);
            break;
         case BOOLEAN:
            yoVariableDefinition.setType(YoType.BooleanYoVariable);
            break;
         case LONG:
            yoVariableDefinition.setType(YoType.LongYoVariable);
            break;
         case ENUM:
            yoVariableDefinition.setType(YoType.EnumYoVariable);

            String[] enumTypes = ((EnumYoVariable<?>) variable).getEnumValuesAsString();

            for (String enumType : enumTypes)
            {
               if(enumType == null)
               {
                  enumType = "null";
               }
               yoVariableDefinition.getEnumValues().add(enumType);
            }
            yoVariableDefinition.setAllowNullValues(((EnumYoVariable<?>) variable).getAllowNullValue());
            break;
         default:
            throw new RuntimeException("Unknown variable type: " + variable.getYoVariableType());
         }

         variableListToPack.add(variable);
         variablesAndRootRegistries.add(new ImmutablePair<YoVariable<?>, YoVariableRegistry>(variable, rootRegistry));
         this.yoVariableIndices.put(variable, variablesAndRootRegistries.size() - 1);

      }

   }

   private void messageFromDynamicGraphicObject(RemoteYoGraphic obj, DynamicGraphicMessage objectMessage)
   {

      objectMessage.setType(obj.getRemoteGraphicType().ordinal());
      objectMessage.setName(obj.getName());

      try
      {
         AppearanceDefinitionMessage appearanceMessage = objectMessage.getAppearance();
         appearanceMessage.setR(obj.getAppearance().getColor().getX());
         appearanceMessage.setG(obj.getAppearance().getColor().getY());
         appearanceMessage.setB(obj.getAppearance().getColor().getZ());
         appearanceMessage.setTransparency(obj.getAppearance().getTransparency());
      }
      catch (NotImplementedException e)
      {
         System.err.println(e.getMessage());
      }

      for (YoVariable<?> yoVar : obj.getVariables())
      {
         Integer index = this.yoVariableIndices.get(yoVar);
         if (index == null)
         {
            throw new RuntimeException("Backing YoVariableRegistry not added for " + obj.getName());
         }
         objectMessage.getYo_index().add(index);
      }

      for (double d : obj.getConstants())
      {
         objectMessage.getConstants().add(d);
      }

   }
   

   public int getNumberOfVariables()
   {
      return variablesAndRootRegistries.size();
   }

   public ArrayList<ImmutablePair<YoVariable<?>, YoVariableRegistry>> getVariablesAndRootRegistries()
   {
      return variablesAndRootRegistries;
   }

   public Handshake getHandShake()
   {
      return handshake;
   }

}

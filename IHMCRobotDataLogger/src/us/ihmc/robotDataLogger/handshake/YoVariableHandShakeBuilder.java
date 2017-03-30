package us.ihmc.robotDataLogger.handshake;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.apache.commons.lang3.NotImplementedException;
import org.apache.commons.lang3.tuple.ImmutablePair;

import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotDataLogger.AppearanceDefinitionMessage;
import us.ihmc.robotDataLogger.EnumType;
import us.ihmc.robotDataLogger.FullStateBuffer;
import us.ihmc.robotDataLogger.GraphicObjectMessage;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.JointDefinition;
import us.ihmc.robotDataLogger.YoRegistryDefinition;
import us.ihmc.robotDataLogger.YoType;
import us.ihmc.robotDataLogger.YoVariableDefinition;
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
   private final TObjectIntHashMap<YoVariable<?>> yoVariableIndices = new TObjectIntHashMap<>();
   private final ArrayList<ImmutablePair<YoVariable<?>, YoVariableRegistry>> variablesAndRootRegistries = new ArrayList<>();
   private final TObjectIntHashMap<Class<?>> enumDescriptions = new TObjectIntHashMap<>(); 
   
   private int registryID = 1;
   private int enumID = 0;

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
               if(handshake.getGraphicObjects().remaining() == 0)
               {
                  throw new RuntimeException("The number of YoGraphics exceeds the maximum amount for the logger (" + handshake.getGraphicObjects().capacity() + ")");
               }
               GraphicObjectMessage msg = handshake.getGraphicObjects().add();
               msg.setListName(yoGraphicsList.getLabel());
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
               if(handshake.getArtifacts().remaining() == 0)
               {
                  throw new RuntimeException("The number of Artifacts exceeds the maximum amount for the logger (" + handshake.getArtifacts().capacity() + ")");
               }
               GraphicObjectMessage msg = handshake.getArtifacts().add();
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

         JointDefinition jointDefinition = handshake.getJoints().add();
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
      
      YoRegistryDefinition yoRegistryDescription = handshake.getRegistries().add();
      yoRegistryDescription.setName("main");
      yoRegistryDescription.setParent((short) 0);
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
      if(myID > handshake.getRegistries().capacity())
      {
         throw new RuntimeException("The number of registries exceeds the maximum number of registries for the logger (" + handshake.getRegistries().capacity() +")");
      }
      registryID++;

      YoRegistryDefinition yoRegistryDescription = handshake.getRegistries().add();
      yoRegistryDescription.setName(registry.getName());
      yoRegistryDescription.setParent((short) parentID);

      addVariables(myID, registry, variableListToPack, rootRegistry);

      for (YoVariableRegistry child : registry.getChildren())
      {
         addRegistry(myID, child, variableListToPack, rootRegistry);
      }

   }
   
   private short getOrAddEnumType(Class<?> enumClass, String[] enumTypes)
   {
      if(enumDescriptions.containsKey(enumClass))
      {
         return (short) enumDescriptions.get(enumClass);
      }
      
      int myID = enumID;
      if(myID > handshake.getEnumTypes().capacity())
      {
         throw new RuntimeException("The number of enum types exceeds the maximum number of enum types for the logger (" + handshake.getEnumTypes().capacity() +")");
      }
      enumID++;
      
      EnumType enumTypeDescription = handshake.getEnumTypes().add();
      String name = enumClass.getCanonicalName();
      if(name.length() > 255)
      {
         name = name.substring(name.length() - 255);
         if(name.startsWith("."))
         {
            name = name.substring(1);
         }
      }
      
      enumTypeDescription.setName(name);
      for (String enumType : enumTypes)
      {
         if(enumType == null)
         {
            enumType = "null";
         }
         enumTypeDescription.getEnumValues().add(enumType);
      }
      
      enumDescriptions.put(enumClass, myID);
      
      return (short) myID;
   }

   private void addVariables(int registryID, YoVariableRegistry registry, List<YoVariable<?>> variableListToPack, YoVariableRegistry rootRegistry)
   {
      ArrayList<YoVariable<?>> variables = registry.getAllVariablesInThisListOnly();
      if(variables.size() > handshake.getVariables().capacity())
      {
         throw new RuntimeException("The number of variables exceeds the maximum number of variables for the logger (" + handshake.getVariables().capacity() +")");
      }
      for (YoVariable<?> variable : variables)
      {
         YoVariableDefinition yoVariableDefinition = handshake.getVariables().add();
         yoVariableDefinition.setName(variable.getName());
         yoVariableDefinition.setRegistry((short) registryID);

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
            if(((EnumYoVariable<?>) variable).isBackedByEnum())
            {
               yoVariableDefinition.setEnumType(getOrAddEnumType(((EnumYoVariable<?>) variable).getEnumType(), ((EnumYoVariable<?>) variable).getEnumValuesAsString()));
            }
            else
            {
               yoVariableDefinition.setEnumType(getOrAddEnumType(variable.getClass(), ((EnumYoVariable<?>) variable).getEnumValuesAsString()));
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

   private void messageFromDynamicGraphicObject(RemoteYoGraphic obj, GraphicObjectMessage objectMessage)
   {

      objectMessage.setType((short) obj.getRemoteGraphicType().ordinal());
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

      if(obj.getVariables().length > objectMessage.getYoVariableIndex().capacity())
      {
         throw new RuntimeException(obj.getName() + " has too many variables. It has " + obj.getVariables().length + " variables");
      }
      
      for (YoVariable<?> yoVar : obj.getVariables())
      {
         if (!this.yoVariableIndices.containsKey(yoVar))
         {
            throw new RuntimeException("Backing YoVariableRegistry not added for " + obj.getName());
         }
         int index = this.yoVariableIndices.get(yoVar);
         objectMessage.getYoVariableIndex().add((short) index);
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

   public void setSummaryProvider(SummaryProvider summaryProvider)
   {
      handshake.getSummary().setCreateSummary(summaryProvider.isSummarize());
      if(summaryProvider.isSummarize())
      {
         handshake.getSummary().setSummaryTriggerVariable(summaryProvider.getSummaryTriggerVariable());
         String[] summarizedVariables = summaryProvider.getSummarizedVariables();
         for(int i = 0; i < summarizedVariables.length; i++)
         {
            String var = summarizedVariables[i];
            handshake.getSummary().getSummarizedVariables().add(var);
         }
      }
   }

}

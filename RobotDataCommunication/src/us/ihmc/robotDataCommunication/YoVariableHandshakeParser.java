package us.ihmc.robotDataCommunication;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import javax.vecmath.Color3f;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Artifact;
import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake;
import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.DynamicGraphicMessage;
import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition;
import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.YoRegistryDefinition;
import us.ihmc.robotDataCommunication.generated.YoProtoHandshakeProto.YoProtoHandshake.YoVariableDefinition;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.utilities.dynamicClassLoader.DynamicEnumCreator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.LongYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.RemoteYoGraphic;
import us.ihmc.yoUtilities.graphics.RemoteYoGraphic.RemoteGraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphic;
import us.ihmc.yoUtilities.graphics.YoGraphicFactory;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;

import com.google.protobuf.InvalidProtocolBufferException;

public class YoVariableHandshakeParser
{

   private final String registryPrefix;
   private final boolean registerYoVariables;
   private YoVariableRegistry registry;
   private double dt;
   private final ArrayList<YoVariableRegistry> registries = new ArrayList<YoVariableRegistry>();
   private final ArrayList<YoVariable<?>> variables = new ArrayList<YoVariable<?>>();
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final ArrayList<JointState<? extends Joint>> jointStates = new ArrayList<>();
   private final DynamicEnumCreator dynamicEnumCreator = new DynamicEnumCreator();
   
   private int numberOfVariables;
   private int numberOfJointStateVariables;
   
   public YoVariableHandshakeParser(String registryPrefix, boolean registerYoVariables)
   {
      this.registerYoVariables = registerYoVariables;
      this.registryPrefix = registryPrefix;
   }

   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   private static YoProtoHandshake parseYoProtoHandshake(byte[] handShake) 
   {
      try
      {
         return YoProtoHandshake.parseFrom(handShake);
      }
      catch (InvalidProtocolBufferException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public static int getNumberOfVariables(byte[] handShake)
   {
      YoProtoHandshake yoProtoHandshake = parseYoProtoHandshake(handShake);
      
      int jointStateVariables = 0;
      for (int i = 0; i < yoProtoHandshake.getJointCount(); i++)
      {
         jointStateVariables += JointState.getNumberOfVariables(yoProtoHandshake.getJoint(i).getType());
      }
      
      return 1 + yoProtoHandshake.getVariableList().size() + jointStateVariables;
   }
   
   public void parseFrom(byte[] handShake)
   {
      YoProtoHandshake yoProtoHandshake = parseYoProtoHandshake(handShake);
      
      this.dt = yoProtoHandshake.getDt();
      if(registerYoVariables)
      {
         YoRegistryDefinition rootDefinition = yoProtoHandshake.getRegistry(0);
         registry = new YoVariableRegistry(registryPrefix + rootDefinition.getName());
         registries.add(registry);
         addChildRegistries(yoProtoHandshake);
   
         addVariables(yoProtoHandshake);
         addJointStates(yoProtoHandshake);
         addGraphicObjects(yoProtoHandshake);
   
      }
      
      this.numberOfVariables = yoProtoHandshake.getVariableCount();
      this.numberOfJointStateVariables = getNumberOfJointStateVariables(yoProtoHandshake);
      
      System.out.println("Receiving " + getNumberOfVariables() + " variables.");
      System.out.println("Receiving " + getNumberOfJointStateVariables() + " joint state variables.");
   }
   
   private int getNumberOfJointStateVariables(YoProtoHandshake yoProtoHandshake)
   {
      int numberOfJointStates = 0;
      for (int i = 0; i < yoProtoHandshake.getJointCount(); i++)
      {
         JointDefinition joint = yoProtoHandshake.getJoint(i);
         numberOfJointStates += JointState.getNumberOfVariables(joint.getType());
      }
      return numberOfJointStates;
   }
   
   private void addJointStates(YoProtoHandshake yoProtoHandshake)
   {
      for (int i = 0; i < yoProtoHandshake.getJointCount(); i++)
      {
         JointDefinition joint = yoProtoHandshake.getJoint(i);
         jointStates.add(JointState.createJointState(joint.getName(), joint.getType()));
      }
   }
   
   

   private void addGraphicObjects(YoProtoHandshake yoProtoHandshake)
   {
      HashMap<String, YoGraphicsList> dgoListMap = new HashMap<String, YoGraphicsList>();
      String listName;
      YoGraphicsList dgoList;
      for (int i = 0; i < yoProtoHandshake.getGraphicObjectCount(); i++)
      {
         listName = "default";
         if (yoProtoHandshake.getGraphicObject(i).hasListName())
         {
            if (!yoProtoHandshake.getGraphicObject(i).getListName().isEmpty())
               listName = yoProtoHandshake.getGraphicObject(i).getListName();
         }

         if (dgoListMap.containsKey(listName))
         {
            dgoList = dgoListMap.get(listName);
         }
         else
         {
            dgoList = new YoGraphicsList(listName);
            dgoListMap.put(listName, dgoList);
         }

         dgoList.add((YoGraphic) getRemoteGraphic(yoProtoHandshake.getGraphicObject(i)));
      }

      for (String list : dgoListMap.keySet())
      {
         yoGraphicsListRegistry.registerYoGraphicsList(dgoListMap.get(list));
      }

      ArtifactList artifactList = new ArtifactList("remote");
      for (int i = 0; i < yoProtoHandshake.getArtifactCount(); i++)
      {
         artifactList.add((Artifact) getRemoteGraphic(yoProtoHandshake.getArtifact(i)));
      }
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   private RemoteYoGraphic getRemoteGraphic(DynamicGraphicMessage msg)
   {
      RemoteGraphicType type = RemoteGraphicType.values()[msg.getType()];
   
      String name = msg.getName();
      YoVariable<?>[] vars = new YoVariable[msg.getYoIndexCount()];
      for (int v = 0; v < vars.length; v++)
         vars[v] = variables.get(msg.getYoIndex(v));
   
      Double[] consts = msg.getConstantList().toArray(new Double[msg.getConstantCount()]);
   
      AppearanceDefinition appearance = new YoAppearanceRGBColor(Color.red, 0.0);
      if (msg.hasAppearance())
      {
         appearance = new YoAppearanceRGBColor(new Color3f((float) msg.getAppearance().getX(), (float) msg.getAppearance().getY(), (float) msg.getAppearance()
               .getZ()), msg.getAppearance().getTransparency());
      }
   
      return YoGraphicFactory.yoGraphicFromMessage(type, name, vars, consts, appearance);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public List<JointState<? extends Joint>> getJointStates()
   {
      return Collections.unmodifiableList(jointStates);
   }

   private void addChildRegistries(YoProtoHandshake yoProtoHandshake)
   {
      for (int i = 1; i < yoProtoHandshake.getRegistryCount(); i++)
      {
         YoRegistryDefinition registryDefinition = yoProtoHandshake.getRegistry(i);
         YoVariableRegistry registry = new YoVariableRegistry(registryDefinition.getName());
         registries.add(registry);
         registries.get(registryDefinition.getParent()).addChild(registry);
      }
   }

   public List<YoVariable<?>> getYoVariablesList()
   {
      return Collections.unmodifiableList(variables);
   }

   private void addVariables(YoProtoHandshake yoProtoHandshake)
   {
      for (YoVariableDefinition yoVariableDefinition : yoProtoHandshake.getVariableList())
      {
         String name = yoVariableDefinition.getName();
         YoVariableRegistry parent = registries.get(yoVariableDefinition.getRegistry());
   
         YoVariable<?> variable;
         switch (yoVariableDefinition.getType())
         {
         case DoubleYoVariable:
            variable = new DoubleYoVariable(name, parent);
            break;
         case IntegerYoVariable:
            variable = new IntegerYoVariable(name, parent);
            break;
         case BooleanYoVariable:
            variable = new BooleanYoVariable(name, parent);
            break;
         case LongYoVariable:
            variable = new LongYoVariable(name, parent);
            break;
         case EnumYoVariable:
            variable = createEnumYoVariable(name, yoVariableDefinition.getEnumValuesList(), parent);
            break;
         default:
            throw new RuntimeException();
         }
   
         variables.add(variable);
      }
   }

   @SuppressWarnings({ "rawtypes", "unchecked" })
   private YoVariable<?> createEnumYoVariable(String name, List<String> values, YoVariableRegistry parent)
   {
      Class<?> enumType = dynamicEnumCreator.createEnum(name, values);
      return new EnumYoVariable(name, "", parent, enumType, true);
   }

   public double getDt()
   {
      return dt;
   }
   
   public YoVariableRegistry getRootRegistry()
   {
      return registry;
   }

   public int getNumberOfVariables()
   {
      return numberOfVariables;
   }

   public int getNumberOfJointStateVariables()
   {
      return numberOfJointStateVariables;
   }

}

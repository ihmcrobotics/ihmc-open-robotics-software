package us.ihmc.robotDataLogger;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import com.google.protobuf.InvalidProtocolBufferException;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.RemoteYoGraphic.RemoteGraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicFactory;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake;
import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake.DynamicGraphicMessage;
import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake.JointDefinition;
import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake.YoRegistryDefinition;
import us.ihmc.robotDataLogger.generated.YoProtoHandshakeProto.YoProtoHandshake.YoVariableDefinition;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.io.printing.PrintTools;

public class YoVariableHandshakeParser
{
   private final String registryPrefix;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final ArrayList<JointState> jointStates = new ArrayList<>();

   private double dt;
   private int bufferSize;
   private List<YoVariableRegistry> registries = new ArrayList<>();
   private List<YoVariable<?>> variables = new ArrayList<>();
   
   public YoVariableHandshakeParser(String registryPrefix)
   {
      this.registryPrefix = registryPrefix;
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
      List<YoVariableRegistry> regs = parseRegistries(yoProtoHandshake, registryPrefix);

      // don't replace those list objects (it's a big code mess), just populate them with received data
      registries.clear();
      registries.addAll(regs);

      List<YoVariable<?>> vars = parseVariables(yoProtoHandshake, regs);

      // don't replace those list objects (it's a big code mess), just populate them with received data
      variables.clear();
      variables.addAll(vars);

      addJointStates(yoProtoHandshake);
      addGraphicObjects(yoProtoHandshake);

      int numberOfVariables = yoProtoHandshake.getVariableCount();
      int numberOfJointStateVariables = getNumberOfJointStateVariables(yoProtoHandshake);
      this.bufferSize = (1 + numberOfVariables + numberOfJointStateVariables) * 8;
   }

   public static List<YoVariableRegistry> parseRegistries(YoProtoHandshake yoProtoHandshake, String registryPrefix)
   {
      YoRegistryDefinition rootDefinition = yoProtoHandshake.getRegistry(0);
      YoVariableRegistry rootRegistry = new YoVariableRegistry(registryPrefix + rootDefinition.getName());

      List<YoVariableRegistry> registryList = new ArrayList<>();
      registryList.add(rootRegistry);

      for (int i = 1; i < yoProtoHandshake.getRegistryCount(); i++)
      {
         YoRegistryDefinition registryDefinition = yoProtoHandshake.getRegistry(i);
         YoVariableRegistry registry = new YoVariableRegistry(registryDefinition.getName());
         registryList.add(registry);
         registryList.get(registryDefinition.getParent()).addChild(registry);
      }

      return registryList;
   }

   @SuppressWarnings("rawtypes")
   public static List<YoVariable<?>> parseVariables(YoProtoHandshake yoProtoHandshake, List<YoVariableRegistry> registryList)
   {
      List<YoVariable<?>> variableList = new ArrayList<>();
      for (YoVariableDefinition yoVariableDefinition : yoProtoHandshake.getVariableList())
      {
         String name = yoVariableDefinition.getName();
         int registryIndex = yoVariableDefinition.getRegistry();
         YoVariableRegistry parent = registryList.get(registryIndex);

         YoVariableDefinition.YoProtoType type = yoVariableDefinition.getType();
         switch (type)
         {
            case DoubleYoVariable:
               DoubleYoVariable doubleVar = new DoubleYoVariable(name, parent);
               variableList.add(doubleVar);
               break;

            case IntegerYoVariable:
               IntegerYoVariable intVar = new IntegerYoVariable(name, parent);
               variableList.add(intVar);
               break;

            case BooleanYoVariable:
               BooleanYoVariable boolVar = new BooleanYoVariable(name, parent);
               variableList.add(boolVar);
               break;

            case LongYoVariable:
               LongYoVariable longVar = new LongYoVariable(name, parent);
               variableList.add(longVar);
               break;

            case EnumYoVariable:
               List<String> values = yoVariableDefinition.getEnumValuesList();
               String[] names = values.toArray(new String[values.size()]);
               boolean allowNullValues = (!yoVariableDefinition.hasAllowNullValues() || yoVariableDefinition.getAllowNullValues());
               EnumYoVariable enumVar = new EnumYoVariable(name, "", parent, allowNullValues, names);
               variableList.add(enumVar);
               break;

            default:
               throw new RuntimeException("Unknown YoVariable type: " + type.name());
         }
      }

      return variableList;
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

         try
         {
            dgoList.add((YoGraphic) getRemoteGraphic(yoProtoHandshake.getGraphicObject(i)));
         }
         catch (Exception e)
         {
            PrintTools.error(this, "Got exception: " + e.getClass().getSimpleName() + " when loading a YoGraphic.");
         }
      }

      for (String list : dgoListMap.keySet())
      {
         yoGraphicsListRegistry.registerYoGraphicsList(dgoListMap.get(list));
      }

      ArtifactList artifactList = new ArtifactList("remote");
      for (int i = 0; i < yoProtoHandshake.getArtifactCount(); i++)
      {
         try
         {
            artifactList.add((Artifact) getRemoteGraphic(yoProtoHandshake.getArtifact(i)));
         }
         catch (Exception e)
         {
            PrintTools.error(this, "Got exception: " + e.getClass().getSimpleName() + " when loading a Artifact.");
         }
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
         appearance = new YoAppearanceRGBColor(new MutableColor((float) msg.getAppearance().getX(), (float) msg.getAppearance().getY(), (float) msg.getAppearance()
               .getZ()), msg.getAppearance().getTransparency());
      }
   
      return YoGraphicFactory.yoGraphicFromMessage(type, name, vars, consts, appearance);
   }

   public YoVariableRegistry getRootRegistry()
   {
      return registries.get(0);
   }

   public List<JointState> getJointStates()
   {
      return Collections.unmodifiableList(jointStates);
   }

   public List<YoVariable<?>> getYoVariablesList()
   {
      return Collections.unmodifiableList(variables);
   }

   public YoGraphicsListRegistry getDynamicGraphicObjectsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public double getDt()
   {
      return dt;
   }

   public int getBufferSize()
   {
      return bufferSize;
   }
}

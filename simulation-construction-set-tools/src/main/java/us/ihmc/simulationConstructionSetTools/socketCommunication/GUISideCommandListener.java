package us.ihmc.simulationConstructionSetTools.socketCommunication;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.simulationconstructionset.gui.CreatedNewRegistriesListener;
import us.ihmc.simulationconstructionset.gui.CreatedNewVariablesListener;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.registry.YoNamespace;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.tools.YoFactories;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class GUISideCommandListener implements GUISideAbstractCommandListener
{
   private final YoRegistry rootRegistry;

   private YoVariableList allVariables;
   private final CreatedNewVariablesListener createdNewVariablesListener;
   private final ReceivedDataListener receivedDataListener;
   private final List<DoDisconnectListener> doDisconnectListeners = new ArrayList<DoDisconnectListener>();
   private final List<CreatedNewRegistriesListener> createdNewRegistryListeners = new ArrayList<CreatedNewRegistriesListener>();

   private final LinkedHashMap<YoVariable, Integer> allVariablesIndexMap = new LinkedHashMap<>();
   private final List<YoVariable> sendVariables = new ArrayList<>();

   private final LinkedHashMap<YoRegistry, Integer> registryIndexMap = new LinkedHashMap<YoRegistry, Integer>();
   private final List<YoRegistry> allRegistries = new ArrayList<YoRegistry>();

   private boolean connected = false;
   private boolean doneReceivingAllRegistriesAndVariables = false;
   private static boolean record = false;

   private int expectedRegistrySettingsIdentifier = Integer.MIN_VALUE;
   private boolean registrySettingsProcessed = false;

   private int numTicks = 0;
   private int logVarsCount;
   YoBuffer dataBuffer;

   public GUISideCommandListener(YoBuffer dataBuffer, YoRegistry rootRegistry, CreatedNewVariablesListener createdNewVariablesListener,
         ReceivedDataListener receivedDataListener)
   {
      this.dataBuffer = dataBuffer;
      this.rootRegistry = rootRegistry;
      this.createdNewVariablesListener = createdNewVariablesListener;
      this.receivedDataListener = receivedDataListener;
   }

   @Override
   public void doHello(String name, String info)
   {
      System.out.println("Received Hello from Robot " + name + ". Message: " + info);
      connected = true;
   }

   @Override
   public void doAllRegistriesAndVariables(String[] registryNames, String[][] variableNames, float[][] initialValues)
   {
      if (registryNames.length != variableNames.length) throw new RuntimeException("registryNames.length != variableNames.length");
      if (registryNames.length != initialValues.length) throw new RuntimeException("registryNames.length != initialValues.length");
      
      doAllRegistries(registryNames, variableNames);

      allVariables = new YoVariableList("All Variables");
      for (int i=0; i<variableNames.length; i++)
      {
         addVariables(variableNames[i].length, variableNames[i], initialValues[i]);
      }
      putVariablesInIndexMap();

//      verifyRegistriesAndVariableListsAreConsistent(registryNames, variableNames);
      doneReceivingAllRegistriesAndVariables = true;
   }

   private void verifyRegistriesAndVariableListsAreConsistent(String[] registryNames, String[][] variableNames)
   {
      for (int i=0; i<registryNames.length; i++)
      {
         String registryName = registryNames[i];
         YoNamespace fullNamespace = new YoNamespace(registryName);
         YoRegistry registry = YoFactories.findOrCreateRegistry(rootRegistry, fullNamespace);

         if (registry.getNumberOfVariables() != variableNames[i].length)
         {
            String error = "registry.getNumberOfYoVariables() = " + registry.getNumberOfVariables() + "!= variableNames[i].length = " + variableNames[i].length;
            error = error + "\nregistry.getName() = " + registry.getName();
            error = error + "\n\n registry variables:\n";

            List<YoVariable> registryVariables = registry.getVariables();
            for (int j=0; j<registryVariables.size(); j++)
            {
               error = error + registryVariables.get(j).getFullNameString() + "\n";
            }

            error = error + "\n\n variableNames:\n";

            for (int j=0; j<variableNames[i].length; j++)
            {
               error = error + variableNames[i][j] + "\n";
            }
            throw new RuntimeException(error);
         }

      }
   }
   
   private void doAllRegistries(String[] registryNames, String[][] variableNames)
   {
      registryIndexMap.clear();
      allRegistries.clear();
      
      int nRegistries = registryNames.length;
      System.out.println("Received " + nRegistries + " registries from robot");

      for (int i = 0; i < nRegistries; i++)
      {
         String registryName = registryNames[i];
         YoNamespace fullNamespace = new YoNamespace(registryName);
         YoRegistry registry = YoFactories.findOrCreateRegistry(rootRegistry, fullNamespace);
         registryIndexMap.put(registry, i);
         allRegistries.add(registry);
         //       System.out.println(i + " " + registry.getNamespace().getName());
      }

      notifyCreatedNewRegistriesListeners();
   }



   private void putVariablesInIndexMap()
   {
      for (int i = 0; i < allVariables.size(); i++)
      {
         YoVariable v = allVariables.get(i);
         allVariablesIndexMap.put(v, i);
         System.out.println(i + " " + v.getFullNameString());
      }
   }

   private void addVariables(int nvars, String[] vars, float[] vals)
   {
      for (int i = 0; i < nvars; i++)
      {
         // System.out.println("Looking for var " + vars[i]);
         String fullVariableName = vars[i];

         YoVariable v = rootRegistry.findVariable(fullVariableName);

         if (allVariables.findVariable(fullVariableName) != null)
         {
            System.err.println("Robot has repeat variable names! Already registered " + fullVariableName);
            System.err.flush();
            System.exit(-1);
         }

         if (v != null)
         {
            addVariableAndSetInitialValue(vals[i], fullVariableName, v);
         }

         else
         {
            createAndAddVariableAndSetInitialValue(vals[i], fullVariableName);
         }
      }
   }

   private void createAndAddVariableAndSetInitialValue(float initialValue, String fullVariableName)
   {
      YoNamespace fullName = new YoNamespace(fullVariableName);
      YoRegistry registry = rootRegistry.findRegistry(fullName.getParent());
      YoDouble newVar = new YoDouble(fullName.getShortName(), "", registry);

      allVariables.add(newVar);

      newVar.set(initialValue);
      System.out.println("Didn't Find var: " + fullVariableName);
      notifyCreatedNewVariablesListeners(newVar);
      
   }

   private void addVariableAndSetInitialValue(float initialValue, String fullVariableName, YoVariable variable)
   {
      System.out.println("Found var: " + fullVariableName);
      allVariables.add(variable);
      variable.setValueFromDouble(initialValue);
   }

   @Override
   public void doRegistrySettingsProcessed(int[] registryIndices, boolean[] isSent, boolean[] isDisallowSendingSet, boolean[] isLogged, int registrySettingsIdentifier)
   {
   }

   public YoVariableList getAllVariables()
   {
      return allVariables;
   }

   public HashMap<YoRegistry, Integer> getRegistryIndexMap()
   {
      return registryIndexMap;
   }

   @Override
   public void doSet(int index, float value)
   {
      throw new RuntimeException("PC should never get this since Robot never sends set...");
   }

   @Override
   public void doPeriod(int periodmsec)
   {
      System.out.println("Period set to " + periodmsec + " milliseconds");
   }

   public void attachDoDisconnectListener(DoDisconnectListener listener)
   {
      doDisconnectListeners.add(listener);
   }

   @Override
   public void doDisconnect()
   {
      connected = false;

      for (DoDisconnectListener listener : doDisconnectListeners)
      {
         listener.doDisconnect();
      }
   }

   @Override
   public void doUserCommand(String command)
   {
      System.out.println("User Command: " + command);
   }

   @Override
   public void doData(float[] data)
   {
      //    System.out.println("Received data");

         if (!(doneReceivingAllRegistriesAndVariables && registrySettingsProcessed))
         return;

      if (!record)
         return;

      if (data.length != sendVariables.size())
      {
         String errorMessage = "GUI Expected data of length " + sendVariables.size() + ", but received data from Robot of size " + data.length;
         System.err.println(errorMessage);
         return; // this happens when you reconnect; the robot is still sending a nonzero number of variables, but we're not initialized yet. 

//         throw new RuntimeException(errorMessage);
      }

      //    System.out.println("Processing data");

      for (int i = 0; i < data.length; i++)
      {
         sendVariables.get(i).setValueFromDouble(data[i]);

         //       System.out.println(sendVariables.get(i));
      }

      receivedDataListener.receivedData(sendVariables);

      //    if (sim != null) sim.tickAndUpdateLeisurely(4); //+++JEP 050722: Make GUI more responsive...
      // myCombinedVarPanel.tickAndUpdate();
      // myGraphArrayPanel.repaintGraphs();

      numTicks++;

      if (numTicks > 50000)
      {
         numTicks = 0;
         
         PrintTools.info(LocalDateTime.now().format(DateTimeFormatter.ISO_DATE_TIME));
      }

   }

   @Override
   public void doTextMessage(String message)
   {
      System.out.println("Text message: " + message);
   }

   public void setRecord(boolean record)
   {

      if (record)
      {
         dataBuffer.setCurrentIndex(dataBuffer.getOutPoint() - 1);
      }
      GUISideCommandListener.record = record;
   }

   public boolean isConnected()
   {
      return connected;
   }

   public static boolean isRecording()
   {
      return record;
   }

   public boolean isDoneReceivingAllRegistriesAndVariables()
   {
      return doneReceivingAllRegistriesAndVariables;
   }

   public void expectNewRegistrySettings(int expectedRegistrySettingsIdentifier)
   {
      registrySettingsProcessed = false;
      this.expectedRegistrySettingsIdentifier = expectedRegistrySettingsIdentifier;
   }

   public boolean getRegistrySettingsProcessed()
   {
      return registrySettingsProcessed;
   }

   public int getIndex(YoVariable variable)
   {
      Integer ret = allVariablesIndexMap.get(variable);

      if (ret == null)
         throw new RuntimeException("AbstractYoVariable not found: " + variable);

      return ret;
   }

   public int getIndex(YoRegistry registry)
   {
      Integer ret = registryIndexMap.get(registry);

      if (ret == null)
         throw new RuntimeException("YoRegistry not found: " + registry.getNamespace());

      return ret;
   }

   private void updateSendVars()
   {
      sendVariables.clear();

      int numberToIterate = allVariables.size();
      
      for (int i=0; i<numberToIterate; i++)
      {
         YoVariable var = allVariables.get(i);
      
         YoRegistry registry = var.getRegistry();
//         if (registry.isSent())
         {
            sendVariables.add(var);
         }
      }
   }

   private void updateLogVarsCount()
   {
      logVarsCount = 0;

      int numberToIterate = allVariables.size();
      
      for (int i=0; i<numberToIterate; i++)
      {
         YoVariable var = allVariables.get(i);
//         if (var.getRegistry().isLogged())
            logVarsCount++;
      }
   }

   public void addCreatedNewRegistryListener(CreatedNewRegistriesListener listener)
   {
      this.createdNewRegistryListeners.add(listener);
   }

   private void notifyCreatedNewRegistriesListeners()
   {
      for (CreatedNewRegistriesListener listener : createdNewRegistryListeners)
      {
         listener.createdNewRegistries();
      }
   }
   
   private void notifyCreatedNewVariablesListeners(YoVariable newVariable)
   {
      createdNewVariablesListener.createdNewVariable(newVariable);
   }

   public int getExpectedRegistrySettingsIdentifier()
   {
      return expectedRegistrySettingsIdentifier;
   }

   public List<YoRegistry> getAllRegistries()
   {
      return allRegistries;
   }

   public int getNumberOfSendVars()
   {
      return sendVariables.size();
   }

   public int getNumberOfLogVars()
   {
      return logVarsCount;
   }
}

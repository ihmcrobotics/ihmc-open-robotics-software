---
title: ValkyrieDemo Run Configuration
sidebar_label: ValkyrieDemo Run Configuration
---

Now we will run the Valkyrie simulation from Eclipse, but the JVM requires more memory.  We suggest at least 4GB.

### Create the Run Configuration from Eclipse

With `ValkyrieDemo.java` open and active in the Java Perspective, choose `Run\Run Configurations...` from the menu.  This will open the Run Configurations dialog. Select `Java Application` on the left as how to run your class, and then click the `New launch configuration` button above it.

![NewRunConfiguration](/img/quickstart/eclipseNewRunConfiguration.png)

You should now have a new Run Configuration under `Java Application` called `ValkyrieDemo`. Choose the `Arguments` tab and add `-Xms4096m -Xmx4096m` to the `VM Arguments:` text area.  Now click `Apply` to save your configuration.

![RunConfigurationVMSettings](/img/quickstart/eclipseVMMemorySetting.png)

You can click `Run` from the `Run Configuration` dialog to run the simulation, or from now on you can click the 'Run' button from the Eclipse Toolbar in the Java Perspective.


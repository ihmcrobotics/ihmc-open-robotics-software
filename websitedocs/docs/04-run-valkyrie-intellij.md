---
title: ValkyrieDemo Run Configuration
sidebar_label: ValkyrieDemo Run Configuration
---

Now we will run the Valkyrie simulation from IntelliJ, but the JVM requires more memory.  We suggest at least 4GB.

### Create the Run Configuration from IntelliJ

Right click on `ValkyrieDemo.java` in your project window, and select  `Run 'ValkyrieDemo.main()'` from the menu.  This will start the simulator and preconfigure the application's run configuration for you.

![NewRunConfiguration](/img/quickstart/intellij/run-valkyrie-demo.png)

The simulation may or may not start up depending on your system, but for now quit the java application window or select the `Stop` button in thelower right.

![NewRunConfiguration](/img/quickstart/intellij/stop-valkyrie-demo.png)

### Edit the Run Configuration

Now select `Run/Edit Configurations...` from the menu to open the `Run/Debug Configurations` dialog.

![RunConfigurationVMSettings](/img/quickstart/intellij/run-configuration.png)

Make sure that `ValkyrieDemo` is selected under `Application`.  Now choose the `Configuration` tab, then in the `VM Options:` text field enter `-Xms4096m -Xmx4096m`.  Finally, select `OK` to save your configuration and close the dialog.

### Run the ValkyrieDemo Simulation

You can now run the ValkyrieDemo simulation by right-clicking `ValkyrieDemo.java` and selecting `Run 'ValkyrieDemo.main()'` or by selecting the run icon ![RunConfigurationVMSettings](/img/quickstart/intellij/run-icon.png) from the menu bar at the top to run the simulation while `ValkyrieDemo` is the currently selected run configuration.



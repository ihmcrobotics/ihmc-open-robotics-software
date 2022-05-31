plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-avatar-interfaces:source") {
      exclude(group = "org.openjfx")
      exclude(group = "org.jmonkeyengine")
      exclude(group = "org.lwjgl.lwjgl") // exclude lwjgl 2
   }
}

javafxDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-javafx-toolkit:0.21.0")
   api("us.ihmc:ihmc-graphics-javafx:source")
   api("us.ihmc:ihmc-graphics-jmonkeyengine:source")
   api("us.ihmc:ihmc-path-planning-visualizers:source")
   api("us.ihmc:robot-environment-awareness-visualizers:source")
}

missionControlDependencies {
   api("us.ihmc:ihmc-commons:0.30.5")
   api("us.ihmc:log-tools:0.6.3")
   api("us.ihmc:ihmc-ros2-library:0.20.5")
   api("us.ihmc:ihmc-java-toolkit:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-interfaces:source")
}

libgdxDependencies {
   api(ihmc.sourceSetProject("main"))
   api(ihmc.sourceSetProject("mission-control"))
   api("org.abego.treelayout:org.abego.treelayout.core:1.0.3")
   api("us.ihmc:ihmc-graphics-libgdx:source")
   api("us.ihmc:ihmc-perception-javacv:source")
   api("us.ihmc:ihmc-graphics-libgdx:source")
   api("us.ihmc:scs2-simulation:0.7.0-bullet-alpha-3")
   api("us.ihmc:mecano-graphviz:0.11.0")
   api("com.badlogicgames.gdx-controllers:gdx-controllers-core:2.2.1")
   api("com.badlogicgames.gdx-controllers:gdx-controllers-desktop:2.2.1")
   api("com.badlogicgames.gdx:gdx-bullet:1.11.0")
   api("com.badlogicgames.gdx:gdx-bullet-platform:1.11.0:natives-desktop")
   api("commons-io:commons-io:2.11.0") // IOUtils method was old version without this
}

testDependencies {
   api(ihmc.sourceSetProject("javafx"))
   api(ihmc.sourceSetProject("libgdx"))
   api("us.ihmc:ihmc-graphics-libgdx-test:source")
   api("us.ihmc:ihmc-path-planning-test:source")
   api("org.cartesiantheatrics:bag-reader-java:0.0.1")
   api("com.github.stephengold:Libbulletjme:12.6.0")
}

app.entrypoint(ihmc.sourceSetProject("mission-control"),
        "MissionControlService",
        "us.ihmc.missionControl.MissionControlService",
        listOf("-Dlog4j2.configurationFile=log4j2NoColor.yml"))
app.entrypoint(ihmc.sourceSetProject("mission-control"),
        "ExampleMissionControlApplication1",
        "us.ihmc.missionControl.ExampleMissionControlApplication1",
        listOf("-Dlog4j2.configurationFile=log4j2NoColor.yml"))
app.entrypoint(ihmc.sourceSetProject("mission-control"),
        "ExampleMissionControlApplication2",
        "us.ihmc.missionControl.ExampleMissionControlApplication2",
        listOf("-Dlog4j2.configurationFile=log4j2NoColor.yml"))

val hostname: String by project
val username: String by project
val distFolder by lazy { ihmc.sourceSetProject("mission-control").tasks.named<Sync>("installDist").get().destinationDir.toString() }

tasks.create("deploy") {
   dependsOn("ihmc-high-level-behaviors-mission-control:installDist")

   doLast {
      remote.session(hostname, username) {
         exec("sudo systemctl stop mission-control-2")

         exec("mkdir -p /home/$username/.ihmc/mission-control")
         exec("sudo mkdir -p /opt/ihmc/mission-control")
         exec("rm -rf /home/$username/.ihmc/mission-control/bin")
         exec("rm -rf /home/$username/.ihmc/mission-control/lib")
         exec("rm -rf /opt/ihmc/mission-control/bin")
         exec("rm -rf /opt/ihmc/mission-control/lib")
         put(file("$distFolder/bin").path, "/home/$username/.ihmc/mission-control/bin")
         put(file("$distFolder/lib").path, "/home/$username/.ihmc/mission-control/lib")
         exec("sudo mv /home/$username/.ihmc/mission-control/bin /opt/ihmc/mission-control/.")
         exec("sudo mv /home/$username/.ihmc/mission-control/lib /opt/ihmc/mission-control/.")
         exec("find /opt/ihmc/mission-control/bin -type f -exec chmod +x {} \\;")

         exec("echo \"${createMissionControlServiceFile(username)}\" > ~/mission-control-2.service")
         exec("sudo mv ~/mission-control-2.service /etc/systemd/system/.")

         exec("echo \"${createExampleApplication1File(username)}\" > ~/mission-control-application-1.service")
         exec("sudo mv ~/mission-control-application-1.service /etc/systemd/system/.")

         exec("echo \"${createExampleApplication2File(username)}\" > ~/mission-control-application-2.service")
         exec("sudo mv ~/mission-control-application-2.service /etc/systemd/system/.")

         exec("sudo systemctl daemon-reload")

         exec("sudo systemctl start mission-control-2")
      }
   }
}

fun createMissionControlServiceFile(username: String): String
{
   return """
      [Unit]
      Description=Mission Control 2 Service
      Wants=network-online.target
      After=network-online.target
   
      [Service]
      User=$username
      AmbientCapabilities=CAP_NET_RAW CAP_NET_ADMIN
      Restart=always
      ExecStart=/opt/ihmc/mission-control/bin/MissionControlService
   
      [Install]
      WantedBy=multi-user.target
   """.trimIndent()
}

fun createExampleApplication1File(username: String): String
{
   return """
      [Unit]
      Description=Mission Control Application 1 Service
      Wants=network-online.target
      After=network-online.target
   
      [Service]
      User=$username
      AmbientCapabilities=CAP_NET_RAW CAP_NET_ADMIN
      Restart=always
      ExecStart=/opt/ihmc/mission-control/bin/ExampleMissionControlApplication1
   
      [Install]
      WantedBy=multi-user.target
   """.trimIndent()
}

fun createExampleApplication2File(username: String): String
{
   return """
      [Unit]
      Description=Mission Control Application 2 Service
      Wants=network-online.target
      After=network-online.target
   
      [Service]
      User=$username
      AmbientCapabilities=CAP_NET_RAW CAP_NET_ADMIN
      Restart=always
      ExecStart=/opt/ihmc/mission-control/bin/ExampleMissionControlApplication2
   
      [Install]
      WantedBy=multi-user.target
   """.trimIndent()
}

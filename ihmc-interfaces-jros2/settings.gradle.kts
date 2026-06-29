pluginManagement {
   plugins {
      id("us.ihmc.ihmc-build") version "1.3.0"
      id("us.ihmc.jros2.generator") version "1.3.0.999"
   }
   repositories {
      maven { url = uri("https://plugins.gradle.org/m2/") }
      maven { url = uri("https://robotlabfiles.ihmc.us/repository/") }
      mavenLocal()
   }
}

buildscript {
   repositories {
      maven { url = uri("https://plugins.gradle.org/m2/") }
      maven { url = uri("https://robotlabfiles.ihmc.us/repository/") }
      mavenLocal()
   }
   dependencies {
      classpath("us.ihmc:ihmc-build:1.3.0")
   }
}

/** Browse source at https://github.com/ihmcrobotics/ihmc-build */
val ihmcSettingsConfigurator = us.ihmc.build.IHMCSettingsConfigurator(settings, logger, extra)
ihmcSettingsConfigurator.checkRequiredPropertiesAreSet()
ihmcSettingsConfigurator.configureExtraSourceSets()
ihmcSettingsConfigurator.findAndIncludeCompositeBuilds()

pluginManagement {
   plugins {
      id("us.ihmc.ihmc-build") version "0.28.6"
   }
}

buildscript {
   repositories {
      maven { url = uri("https://plugins.gradle.org/m2/") }
      mavenLocal()
   }
   dependencies {
      classpath("us.ihmc:ihmc-build:0.28.6")
   }
}

/**
 * Browse source at https://github.com/ihmcrobotics/ihmc-build
 */
val ihmcSettingsConfigurator = us.ihmc.build.IHMCSettingsConfigurator(settings, logger, extra)
ihmcSettingsConfigurator.checkRequiredPropertiesAreSet()
ihmcSettingsConfigurator.configureExtraSourceSets()
ihmcSettingsConfigurator.findAndIncludeCompositeBuilds()

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   loadProductProperties("../product.properties")
   
   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("org.apache.xmlgraphics:batik-dom:1.14")

   api("us.ihmc:ihmc-avatar-interfaces-visualizers:source")
   api("us.ihmc:robotiq-hand-drivers:source")
   api("us.ihmc:ihmc-model-file-loader:source")
   api("us.ihmc:ihmc-manipulation-planning:source")
   api("us.ihmc:ihmc-parameter-tuner:0.14.1")
   api("us.ihmc:ihmc-footstep-planning-visualizers:source")
   api("us.ihmc:ihmc-high-level-behaviors:source")
}

testDependencies {
   api("us.ihmc:ihmc-avatar-interfaces-test:source")
   api("us.ihmc:ihmc-sensor-processing-test:source")
   api("us.ihmc:ihmc-simulation-toolkit-test:source")
   api("us.ihmc:ihmc-messager-test:0.2.0")
}

// These are to configure the atlas tests so a new JVM instance is created for each test.
// Simulations use a lot of heap memory and have memory leaks.
categories.configure("fast")
{
   forkEvery = 1
}

categories.configure("video")
{
   forkEvery = 1
}

categories.configure("controller-api")
{
   forkEvery = 1
}

categories.configure("controller-api-2")
{
   forkEvery = 1
}

categories.configure("humanoid-behaviors")
{
   forkEvery = 1
}

categories.configure("humanoid-flat-ground")
{
   forkEvery = 1
}

categories.configure("humanoid-flat-ground-bullet")
{
   forkEvery = 1
}

categories.configure("humanoid-obstacle")
{
   forkEvery = 1
}

categories.configure("humanoid-obstacle-2")
{
   forkEvery = 1
}

categories.configure("humanoid-push-recovery")
{
   forkEvery = 1
}

categories.configure("humanoid-rough-terrain")
{
   forkEvery = 1
}

categories.configure("humanoid-toolbox")
{
   forkEvery = 1
}